#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Low-latency camera streaming server.

Protocol per frame:
    [8-byte network-order double: server_timestamp]
    [4-byte network-order uint32: jpeg_payload_size]
    [jpeg_payload_size bytes: JPEG frame]

Latency-oriented changes relative to the original version:
- Continuously drain the camera into a single latest-frame slot.
- Drop intermediate frames instead of letting send/encode latency build a queue.
- Disable the preview window by default because GUI refresh adds delay.
- Avoid concatenating ``header + payload`` for every frame.
- Keep listening for the next client after a disconnect.
"""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np


# Fixed 12-byte network header: 8-byte timestamp + 4-byte JPEG payload length.
_HEADER_STRUCT = struct.Struct("!dI")
_PREVIEW_WINDOW_NAME = "server preview"
_DISCONNECT_ERRORS = (
    BrokenPipeError,
    ConnectionResetError,
    ConnectionAbortedError,
    TimeoutError,
    socket.timeout,
)


def _require_positive_int(name, value):# -> int:
    """Validate integer parameters early so failures are explicit."""
    if not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {type(value).__name__}")
    if value <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")
    return value


def _require_nonnegative_int(name, value):# -> int:
    """Validate non-negative integer parameters."""
    if not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {type(value).__name__}")
    if value < 0:
        raise ValueError(f"{name} must be >= 0, got {value}")
    return value


def _require_positive_float(name, value):# -> float:
    """Validate positive float parameters when timeouts are enabled."""
    if not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number, got {type(value).__name__}")
    value = float(value)
    if value <= 0.0:
        raise ValueError(f"{name} must be > 0, got {value}")
    return value


def _open_camera(camera_index, use_v4l2):# -> cv2.VideoCapture:
    """Open the camera with a low-latency backend preference on Linux."""
    open_attempts = []

    # Prefer V4L2 on Linux because it usually exposes useful low-latency controls.
    if use_v4l2 and sys.platform.startswith("linux") and hasattr(cv2, "CAP_V4L2"):
        open_attempts.append((camera_index, cv2.CAP_V4L2))

    # Always fall back to the default backend.
    open_attempts.append((camera_index, cv2.CAP_ANY))

    for index, backend in open_attempts:
        cap = None
        try:
            # Some OpenCV builds accept ``backend`` in the constructor.
            cap = cv2.VideoCapture(index, backend)
        except TypeError:
            cap = cv2.VideoCapture(index)

        if cap is not None and cap.isOpened():
            return cap

        if cap is not None:
            cap.release()

    raise RuntimeError(f"Could not open camera index {camera_index}")


def _configure_camera(
    cap,
    width,
    height,
    fps,
    camera_fourcc,
):# -> None:
    """Apply best-effort low-latency camera settings.

    Many camera drivers ignore unsupported properties. That is fine here because
    these calls are opportunistic rather than mandatory.
    """
    assert cap.isOpened(), "Camera must be opened before configuration"

    # Geometry and FPS are advisory on many drivers, but setting them early helps.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))

    # Ask the backend to keep as little buffering as possible.
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # MJPG frequently reduces USB bandwidth and camera-side buffering when supported.
    if camera_fourcc:
        if len(camera_fourcc) != 4:
            raise ValueError("camera_fourcc must be exactly 4 characters, for example 'MJPG'")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*camera_fourcc))


class LatestFrameGrabber:
    """Continuously capture frames and keep only the newest one.

    This prevents latency growth when JPEG encoding or network I/O occasionally
    takes longer than a single frame interval.
    """

    def __init__(
        self,
        camera_index,
        width,
        height,
        fps,
        use_v4l2,
        camera_fourcc,
    ):# -> None:
        self.camera_index = _require_nonnegative_int("camera_index", camera_index)
        self.width = _require_positive_int("width", width)
        self.height = _require_positive_int("height", height)
        self.fps = _require_positive_int("fps", fps)
        self.use_v4l2 = bool(use_v4l2)
        self.camera_fourcc = camera_fourcc

        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._condition = threading.Condition()

        # Shared frame state. ``_latest_frame`` is replaced atomically with a new
        # NumPy array on every successful capture.
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_id = 0
        self._running = False
        self._capture_exception: Optional[BaseException] = None

    def start(self):# -> None:
        """Open the camera and launch the background capture thread."""
        if self._thread is not None:
            raise RuntimeError("LatestFrameGrabber.start() called more than once")

        self._cap = _open_camera(self.camera_index, self.use_v4l2)
        _configure_camera(self._cap, self.width, self.height, self.fps, self.camera_fourcc)

        self._running = True
        self._thread = threading.Thread(
            target=self._worker,
            name="camera-capture",
            daemon=True,
        )
        self._thread.start()

    def stop(self):# -> None:
        """Stop the capture worker and release camera resources."""
        self._stop_event.set()

        # Releasing the capture handle helps unblock ``read()`` on some backends.
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def wait_for_new_frame(
        self,
        last_seen_frame_id,
        timeout_s = 1.0,
    ):# -> Tuple[Optional[np.ndarray], int]:
        """Wait until a fresher frame is available.

        Returns
        -------
        frame, frame_id
            ``frame`` is ``None`` on timeout. Otherwise it is the newest frame
            available at wake-up time, and ``frame_id`` is its monotonically
            increasing sequence number.
        """
        timeout_s = _require_positive_float("timeout_s", timeout_s)
        deadline = time.monotonic() + timeout_s

        with self._condition:
            while (
                self._running
                and self._latest_frame_id == last_seen_frame_id
                and self._capture_exception is None
            ):
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    return None, last_seen_frame_id
                self._condition.wait(timeout=remaining_s)

            if self._capture_exception is not None:
                raise RuntimeError("Camera capture worker failed") from self._capture_exception

            if self._latest_frame is None or self._latest_frame_id == last_seen_frame_id:
                return None, last_seen_frame_id

            return self._latest_frame, self._latest_frame_id

    def _worker(self):# -> None:
        """Capture loop that continuously overwrites the single latest frame."""
        consecutive_failures = 0

        try:
            while not self._stop_event.is_set():
                assert self._cap is not None, "Camera handle must exist while capture thread runs"
                ok, frame = self._cap.read()

                if not ok:
                    consecutive_failures += 1
                    if consecutive_failures == 1 or consecutive_failures % 60 == 0:
                        print(f"[SERVER] Camera read failed ({consecutive_failures})")
                    time.sleep(0.005)
                    continue

                consecutive_failures = 0

                with self._condition:
                    self._latest_frame = frame
                    self._latest_frame_id += 1
                    self._condition.notify_all()
        except BaseException as exc:
            self._capture_exception = exc
        finally:
            self._running = False
            with self._condition:
                self._condition.notify_all()

            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None


def _configure_client_socket(conn, send_timeout_s):# -> None:
    """Configure the connected client socket for low-latency streaming."""
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

    if send_timeout_s is not None:
        conn.settimeout(_require_positive_float("send_timeout_s", send_timeout_s))


def _send_memoryviews(conn, views):# -> None:
    """Send multiple buffers without building a concatenated temporary payload.

    ``socket.sendmsg`` performs scatter/gather I/O on Unix-like platforms. When it
    is unavailable, falling back to ``sendall`` still avoids the large ``header +
    payload`` allocation present in the original server.
    """
    pending = [view for view in views if len(view) > 0]

    if hasattr(conn, "sendmsg"):
        while pending:
            sent = conn.sendmsg(pending)
            if sent == 0:
                raise ConnectionResetError("socket closed during sendmsg")

            while pending and sent >= len(pending[0]):
                sent -= len(pending[0])
                pending.pop(0)

            if pending and sent > 0:
                pending[0] = pending[0][sent:]
        return

    for view in pending:
        conn.sendall(view)


def _send_packet(
    conn,
    header_buffer,
    timestamp_s,
    encoded_jpeg,
):# -> None:
    """Pack and send one frame packet.

    Parameters
    ----------
    encoded_jpeg:
        One-dimensional ``uint8`` array produced by ``cv2.imencode``.
    """
    assert encoded_jpeg.ndim == 1, "JPEG buffer must be a flat array"
    assert encoded_jpeg.dtype == np.uint8, "JPEG buffer must use uint8 storage"

    frame_size = int(encoded_jpeg.size)
    _HEADER_STRUCT.pack_into(header_buffer, 0, float(timestamp_s), frame_size)

    header_view = memoryview(header_buffer)
    payload_view = memoryview(encoded_jpeg)
    _send_memoryviews(conn, [header_view, payload_view])


def _maybe_show_preview(frame, show_preview):# -> bool:
    """Display the preview window only when explicitly requested.

    Returns ``False`` when the user pressed ``q`` to stop the server.
    """
    if not show_preview:
        return True

    cv2.imshow(_PREVIEW_WINDOW_NAME, frame)
    key = cv2.waitKey(1) & 0xFF
    return key != ord("q")


def _serve_client(
    conn,
    frame_source,
    encode_params,
    show_preview,
):# -> bool:
    """Stream the newest available frames to one client.

    Returns
    -------
    bool
        ``True`` to continue serving future clients after this client disconnects.
        ``False`` to terminate the whole server, typically because the operator
        pressed ``q`` in the optional preview window.
    """
    header_buffer = bytearray(_HEADER_STRUCT.size)
    last_sent_frame_id = -1

    while True:
        frame, frame_id = frame_source.wait_for_new_frame(last_sent_frame_id, timeout_s=1.0)

        # Timeouts are expected if the camera momentarily stalls. Keep waiting.
        if frame is None:
            continue

        if not _maybe_show_preview(frame, show_preview):
            return False

        # Timestamp as close as possible to the encode/send boundary.
        timestamp_s = time.time()

        success, encoded = cv2.imencode(".jpg", frame, encode_params)
        if not success:
            print("[SERVER] JPEG encoding failed")
            last_sent_frame_id = frame_id
            continue

        _send_packet(conn, header_buffer, timestamp_s, encoded)
        last_sent_frame_id = frame_id


def start_server(
    host="0.0.0.0",
    port = 5000,
    camera_index = 0,
    width = 640,
    height = 480,
    fps = 30,
    jpeg_quality = 80,
    show_preview = False,
    use_v4l2 = True,
    camera_fourcc = "MJPG",
    send_timeout_s= 2.0,
):# -> None:
    """Stream frames from a USB camera over TCP with timestamps.

    The server keeps the camera alive in a background capture thread and accepts
    clients sequentially. When the current client disconnects, the server returns
    to ``accept()`` and waits for the next one.
    """
    _require_positive_int("port", port)
    _require_nonnegative_int("camera_index", camera_index)
    _require_positive_int("width", width)
    _require_positive_int("height", height)
    _require_positive_int("fps", fps)
    jpeg_quality = _require_positive_int("jpeg_quality", jpeg_quality)
    if jpeg_quality > 100:
        raise ValueError(f"jpeg_quality must be <= 100, got {jpeg_quality}")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    frame_source = LatestFrameGrabber(
        camera_index=camera_index,
        width=width,
        height=height,
        fps=fps,
        use_v4l2=use_v4l2,
        camera_fourcc=camera_fourcc,
    )

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        frame_source.start()
        server_sock.bind((host, port))
        server_sock.listen(1)
        print(f"[SERVER] Listening on {host}:{port}...")

        while True:
            conn, addr = server_sock.accept()
            print(f"[SERVER] Client connected from {addr}")

            try:
                _configure_client_socket(conn, send_timeout_s)
                keep_running = _serve_client(conn, frame_source, encode_params, show_preview)
            except _DISCONNECT_ERRORS as exc:
                print(f"[SERVER] Client disconnected: {exc}")
                keep_running = True
            except OSError as exc:
                print(f"[SERVER] Socket error while streaming: {exc}")
                keep_running = True
            finally:
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                conn.close()

            if not keep_running:
                print("[SERVER] Stop requested from preview window")
                break

            print("[SERVER] Waiting for a new client...")
    except KeyboardInterrupt:
        print("[SERVER] Interrupted by user")
    finally:
        frame_source.stop()
        server_sock.close()
        cv2.destroyAllWindows()
        print("[SERVER] Shutdown complete")


if __name__ == "__main__":
    # Adjust host, port, and camera settings as needed.
    start_server(
        host="0.0.0.0",
        port=5000,
        camera_index=1,
        width=1920,
        height=1080,
        fps=30,
        jpeg_quality=80,
        show_preview=False,
    )
