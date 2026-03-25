#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Low-latency camera feed client with ROS topic overlays and a cached right panel.

This version is optimized for the undelayed / latest-frame case:
1. The network reader runs in a background thread and keeps only the newest JPEG.
   Older frames are overwritten before decode, so latency does not grow unbounded.
2. The static right-side map panel is rendered once per display size and reused.
3. The timing text background is blended only in a small ROI, not over the full frame.
4. Socket reads avoid repeated bytes concatenation.
5. Optional reduced-resolution JPEG decode is used when the requested scale is an
   exact 1/2, 1/4, or 1/8. This lets OpenCV decode fewer pixels and skip a resize.

Compared with the original script, this trades perfect frame delivery for low latency.
That is usually the correct choice for a live operator feed.
"""

import argparse
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

# Try to import rospy and std_msgs; if unavailable, run without ROS overlays.
try:
    import rospy
    from std_msgs.msg import Bool
    ROS_AVAILABLE = True
except Exception:
    rospy = None
    Bool = None
    ROS_AVAILABLE = False

# -------------------------
# User-configurable globals
# -------------------------
goal_reached_topic = "/goal_reached"
is_collision_topic = "/is_collision"

# Keep the original defaults unless the CLI overrides them.
frame_rotation = 270.0
frame_scaling = 0.6
right_canvas_aspect = 3.0 / 2.0

# For the undelayed feed, default to no intentional display delay.
# total_delay = 1.0
total_delay = 0.001


OVERLAY_ALPHA = 0.35
GOAL_COLOR = (0, 200, 0)
COLLISION_COLOR = (0, 0, 200)

# If the display scale exactly matches one of OpenCV's reduced JPEG decode modes,
# decode fewer pixels and skip a later resize. This materially reduces CPU load.
USE_REDUCED_JPEG_DECODE = True

# The current script always uses the medium panel layout.
DEFAULT_LAYOUT = "medium"
PANEL_PADDING_PX = 12

# -------------------------
# Thread-safe flags for ROS callbacks
# -------------------------
_flags_lock = threading.Lock()
_goal_reached_flag = False
_collision_flag = False


def set_goal_flag(value):
    """Thread-safe setter for goal flag."""
    assert isinstance(value, (bool, int)), "Goal flag value must be bool or int"
    global _goal_reached_flag
    # with _flags_lock:
        # _goal_reached_flag = bool(value)
    _goal_reached_flag = bool(value)


def set_collision_flag(value):
    """Thread-safe setter for collision flag."""
    assert isinstance(value, (bool, int)), "Collision flag value must be bool or int"
    global _collision_flag
    # with _flags_lock:
    #     _collision_flag = bool(value)
    _collision_flag = bool(value)


def get_flags(): #-> Tuple[bool, bool]:
    """Thread-safe getter returning (goal_reached, collision)."""
    # with _flags_lock:
    #     return bool(_goal_reached_flag), bool(_collision_flag)
    # return bool(_goal_reached_flag), bool(_collision_flag)
    return bool(_goal_reached_flag), bool(_collision_flag)

# -------------------------
# ROS callbacks (if available)
# -------------------------
def _on_goal_msg(msg): # -> None:
    """Callback for goal_reached_topic (expects std_msgs/Bool)."""
    try:
        set_goal_flag(bool(msg.data))
    except Exception:
        set_goal_flag(False)
        # set_goal_flag(True)



def _on_collision_msg(msg):# -> None:
    """Callback for is_collision_topic (expects std_msgs/Bool)."""
    try:
        set_collision_flag(bool(msg.data))
    except Exception:
        set_collision_flag(False)


# -------------------------
# Static panel helpers
# -------------------------
def load_map_images(image_dir = "./"):# -> Dict[str, np.ndarray]:
    """Load all map panel images once from disk."""
    assert isinstance(image_dir, str), "image_dir must be a string"
    image_dict: Dict[str, np.ndarray] = {}
    for layout in ["easy", "medium", "hard"]:
        path = f"{image_dir}map_{layout}.png"
        cached_panel_image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if cached_panel_image is None:
            raise RuntimeError(f"Failed to load panel image: {path}")
        image_dict[layout] = cached_panel_image
    return image_dict



def build_static_right_panel(
    source_image,
    canvas_height,
    *,
    aspect,
    padding_px = PANEL_PADDING_PX,
):# -> np.ndarray:
    """
    Render the static map panel once for a given display height.

    The result is cached and reused for all subsequent frames with the same height.
    This removes the dominant per-frame cost in the original script.
    """
    assert isinstance(source_image, np.ndarray), "source_image must be a numpy array"
    assert source_image.ndim == 3, "source_image must have shape (H, W, C)"
    assert source_image.shape[2] in (3, 4), "source_image must be BGR or BGRA"
    assert canvas_height > 0 and isinstance(canvas_height,int), "canvas_height must be positive"
    assert aspect > 0.0, "aspect must be positive"
    assert padding_px >= 0, "padding_px must be non-negative"

    # Output panel has fixed light-gray background and a subtle border.
    canvas_width = max(1, int(round(canvas_height * aspect)))
    panel = np.full((canvas_height, canvas_width, 3), 240, dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (canvas_width - 1, canvas_height - 1), (200, 200, 200), 1)

    inner_height = canvas_height - 2 * padding_px
    inner_width = canvas_width - 2 * padding_px
    assert inner_height > 0 and inner_width > 0, "Padding leaves no drawable panel region"

    # Normalize the source image to BGR plus optional alpha.
    if source_image.shape[2] == 4:
        src_bgr = source_image[:, :, :3]
        src_alpha = source_image[:, :, 3]
    else:
        src_bgr = source_image
        src_alpha = None

    src_height, src_width = src_bgr.shape[:2]
    assert src_height > 0 and src_width > 0, "Source panel image has invalid dimensions"

    # Fit the panel image into the available area while preserving aspect ratio.
    scale = min(inner_width / float(src_width), inner_height / float(src_height))
    new_width = max(1, int(round(src_width * scale)))
    new_height = max(1, int(round(src_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR

    resized_bgr = cv2.resize(src_bgr, (new_width, new_height), interpolation=interpolation)
    x0 = padding_px + max(0, (inner_width - new_width) // 2)
    y0 = padding_px + max(0, (inner_height - new_height) // 2)
    x1 = x0 + new_width
    y1 = y0 + new_height

    roi = panel[y0:y1, x0:x1]
    assert roi.shape[:2] == (new_height, new_width), "Panel ROI shape mismatch"

    if src_alpha is None:
        roi[:] = resized_bgr
    else:
        resized_alpha = cv2.resize(src_alpha, (new_width, new_height), interpolation=interpolation)
        alpha_f = (resized_alpha.astype(np.float32) / 255.0)[:, :, None]
        src_f = resized_bgr.astype(np.float32)
        dst_f = roi.astype(np.float32)
        roi[:] = np.clip(src_f * alpha_f + dst_f * (1.0 - alpha_f), 0.0, 255.0).astype(np.uint8)

    return panel


# -------------------------
# Image transform helpers
# -------------------------
_rotation_matrix_cache: Dict[Tuple[int, int, float], np.ndarray] = {}


def rotate_frame_cached(frame, degrees):# -> np.ndarray:
    """
    Rotate an image about its center by `degrees`.

    This preserves the original output size, matching the original script's
    semantics. The affine matrix is cached by (H, W, degrees) to avoid small but
    unnecessary per-frame setup overhead.
    """
    assert isinstance(degrees, (int, float)), "degrees must be numeric"
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    if degrees % 360 == 0:
        return frame

    height, width = frame.shape[:2]
    cache_key = (height, width, float(degrees))
    M = _rotation_matrix_cache.get(cache_key)
    if M is None:
        center = (width / 2.0, height / 2.0)
        M = cv2.getRotationMatrix2D(center, degrees, 1.0)
        _rotation_matrix_cache[cache_key] = M

    rotated = cv2.warpAffine(frame, M, (width, height), flags=cv2.INTER_LINEAR)
    return rotated



def scale_frame(frame, scale):# -> np.ndarray:
    """Scale an image by `scale`. Scale must be strictly positive."""
    assert isinstance(scale, (int, float)), "scale must be numeric"
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    assert scale > 0.0, "scale must be > 0"
    if abs(scale - 1.0) < 1e-9:
        return frame

    height, width = frame.shape[:2]
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))

    # Use INTER_AREA for downscaling and INTER_LINEAR for upscaling.
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(frame, (new_width, new_height), interpolation=interpolation)



def maybe_decode_reduced_jpeg(scale):# -> Tuple[int, float]:
    """
    Select an OpenCV JPEG decode mode and the effective built-in scale factor.

    OpenCV can decode JPEGs directly to 1/2, 1/4, or 1/8 resolution. Using this
    avoids spending CPU on pixels that will immediately be thrown away.
    """
    assert isinstance(scale, (int, float)), "scale must be numeric"
    if not USE_REDUCED_JPEG_DECODE:
        return cv2.IMREAD_COLOR, 1.0

    if abs(scale - 0.5) < 1e-9:
        return cv2.IMREAD_REDUCED_COLOR_2, 0.5
    if abs(scale - 0.25) < 1e-9:
        return cv2.IMREAD_REDUCED_COLOR_4, 0.25
    if abs(scale - 0.125) < 1e-9:
        return cv2.IMREAD_REDUCED_COLOR_8, 0.125
    return cv2.IMREAD_COLOR, 1.0



def apply_overlay_tint_inplace(
    frame,
    color,
    alpha,
    solid_cache,
):# -> None:
    """
    Blend a uniform color over the entire frame in place.

    The solid color image is cached per frame shape and color. This avoids
    repeatedly allocating and filling a full-size overlay.
    """
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    assert isinstance(color, tuple) and len(color) == 3 and all(isinstance(c, int) for c in color), "color must be a tuple of 3 ints"
    assert isinstance(alpha, (int, float)), "alpha must be numeric"
    assert isinstance(solid_cache, dict), "solid_cache must be a dictionary"
    assert frame.ndim == 3 and frame.shape[2] == 3, "frame must be HxWx3 BGR"
    assert 0.0 <= alpha <= 1.0, "alpha must be between 0 and 1"

    cache_key = (frame.shape[0], frame.shape[1], color)
    solid = solid_cache.get(cache_key)
    if solid is None:
        solid = np.empty_like(frame)
        solid[:] = color
        solid_cache[cache_key] = solid

    cv2.addWeighted(solid, alpha, frame, 1.0 - alpha, 0.0, frame)



def darken_text_background_inplace(
    frame: np.ndarray, *, x0, y0, width, height, alpha,
):# -> None:
    """
    Darken only the small text ROI in place.

    The original script copied the entire frame twice and blended over the whole
    image just to darken a small text background. This function confines the work
    to that small rectangle.
    """
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    assert isinstance(x0, int) and isinstance(y0, int), "x0 and y0 must be integers"
    assert isinstance(width, int) and isinstance(height, int), "width and height must be integers"
    assert isinstance(alpha, (int, float)), "alpha must be numeric"
    assert 0.0 <= alpha <= 1.0, "alpha must be between 0 and 1"
    assert width > 0 and height > 0, "Text ROI must be positive"

    x1 = max(0, x0)
    y1 = max(0, y0)
    x2 = min(frame.shape[1], x0 + width)
    y2 = min(frame.shape[0], y0 + height)
    if x2 <= x1 or y2 <= y1:
        return

    roi = frame[y1:y2, x1:x2]
    # Blending with black is equivalent to scaling the pixel intensities by (1-alpha).
    cv2.convertScaleAbs(roi, dst=roi, alpha=(1.0 - alpha), beta=0.0)



def delayed_frame_render(now, frame_buffer):
    """Check the frame buffer for the closest frame overdue
    (due to specified delay) for display at `now`."""
    last_rem_time = 1
    for i, frame_item in enumerate(frame_buffer):
        frame_ts, frame = frame_item
        elapsed = now - frame_ts
        rem_time = total_delay - elapsed
        # print(f' {i}:{round(rem_time,2)}', end='')

        if rem_time > 0 and last_rem_time < 0:

            render_frame = frame_buffer[i-1][1]
            frame_buffer = frame_buffer[i:]
            cv2.imshow("Camera Feed", render_frame)
            # print(' [found]')
            return render_frame, frame_buffer
        last_rem_time = rem_time

        # print(f"Frame {i}:  rem_time={rem_time:.2f}")
    # no delayed frame available
    # print('')
    return None, frame_buffer

def draw_timing_overlay_inplace(
    frame, net_ms, added_ms, total_ms,
):# -> None:
    """Draw timing text on the frame in place."""
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    assert isinstance(net_ms, (int, float)), "net_ms must be numeric"
    assert isinstance(added_ms, (int, float)), "added_ms must be numeric"
    assert isinstance(total_ms, (int, float)), "total_ms must be numeric"
    assert frame.ndim == 3 and frame.shape[2] == 3, "frame must be HxWx3 BGR"

    text_lines = [
        f"Network: {net_ms:7.1f} ms",
        f"Added:   {added_ms:7.1f} ms",
        f"Total:   {total_ms:7.1f} ms",
    ]

    x = 10
    y0 = 30
    dy = 25
    rect_x = x - 5
    rect_y = 5
    rect_w = 260
    rect_h = dy * len(text_lines) + 10

    darken_text_background_inplace(
        frame,
        x0=rect_x,
        y0=rect_y,
        width=rect_w,
        height=rect_h,
        alpha=0.4,
    )

    for i, line in enumerate(text_lines):
        y = y0 + i * dy
        cv2.putText(
            frame,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


# -------------------------
# Networking helpers
# -------------------------
def recv_exact(sock, count):# -> bytearray:
    """
    Receive exactly `count` bytes from the socket, raising on EOF.

    This uses recv_into + a preallocated bytearray to avoid repeated bytes
    concatenation. That reduces allocations and copies on the receive path.
    """
    assert isinstance(sock, socket.socket), "sock must be a socket.socket instance"
    assert isinstance(count, int) and count >= 0, "count must be a non-negative integer"
    assert count >= 0, "count must be non-negative"

    buf = bytearray(count)
    view = memoryview(buf)
    received = 0
    while received < count:
        nbytes = sock.recv_into(view[received:], count - received)
        if nbytes == 0:
            raise ValueError("Socket closed while receiving data")
        received += nbytes
    return buf


@dataclass
class LatestJpegPacket:
    """Newest received packet. Older packets are intentionally overwritten."""
    seq: int
    server_ts: float
    arrival_ts: float
    jpg_bytes: bytearray


class LatestPacketStore:
    """
    Thread-safe single-slot buffer that keeps only the newest packet.

    This is the key latency optimization. If the UI falls behind, stale frames are
    discarded instead of queued. A live operator feed should prefer freshness over
    complete delivery.
    """

    def __init__(self):# -> None:
        self._cond = threading.Condition()
        self._latest: Optional[LatestJpegPacket] = None
        self._seq = 0
        self._closed = False
        self._error: Optional[BaseException] = None

    def put(self, server_ts, arrival_ts, jpg_bytes):# -> None:
        assert isinstance(jpg_bytes, bytearray), "jpg_bytes must be a bytearray"
        with self._cond:
            self._seq += 1
            self._latest = LatestJpegPacket(
                seq=self._seq,
                server_ts=float(server_ts),
                arrival_ts=float(arrival_ts),
                jpg_bytes=jpg_bytes,
            )
            self._cond.notify_all()

    def set_closed(self, error = None):# -> None:
        with self._cond:
            self._closed = True
            self._error = error
            self._cond.notify_all()

    def wait_for_newer(self, last_seq, timeout_sec = 0.01):# -> Optional[LatestJpegPacket]:
        """Block briefly until a newer packet arrives, then return it."""
        assert timeout_sec >= 0.0, "timeout_sec must be non-negative"

        with self._cond:
            if self._latest is None or self._latest.seq <= last_seq:
                self._cond.wait(timeout=timeout_sec)

            if self._latest is not None and self._latest.seq > last_seq:
                return self._latest

            if self._closed and self._error is not None:
                raise RuntimeError(f"Receiver thread failed: {self._error}") from self._error

            return None

    @property
    def closed(self):# -> bool:
        with self._cond:
            return self._closed



def receiver_thread_main(sock, latest_store, stop_event):# -> None:
    """Continuously read framed JPEG packets and keep only the newest one."""
    assert isinstance(sock, socket.socket), "sock must be a socket.socket instance"
    assert isinstance(latest_store, LatestPacketStore), "latest_store must be a LatestPacketStore instance"
    assert isinstance(stop_event, threading.Event), "stop_event must be a threading.Event instance"
    try:
        while not stop_event.is_set():
            header = recv_exact(sock, 8 + 4)
            server_ts, frame_size = struct.unpack("!dI", header)
            if frame_size <= 0:
                raise ValueError(f"Received invalid frame size: {frame_size}")

            jpg_bytes = recv_exact(sock, frame_size)
            arrival_ts = time.time()
            latest_store.put(server_ts, arrival_ts, jpg_bytes)
    except Exception as exc:
        latest_store.set_closed(error=exc)





# -------------------------
# Main client
# -------------------------
def start_client(
    server_host,
    server_port,
    *,
    total_delay_override= None,
    goal_reached_topic_name = None,
    is_collision_topic_name = None,
    rotation_degrees = None,
    scaling_factor = None,
    right_aspect = None,
    panel_layout = DEFAULT_LAYOUT,
    use_ros = True,
):# -> None:
    """
    Connect to the server and display the newest available frame with minimal lag.

    The display loop intentionally operates in latest-frame mode. It decodes only
    the freshest JPEG received so far and drops stale frames automatically.
    """
    global total_delay, frame_rotation, frame_scaling, right_canvas_aspect



    frame_buffer = []
    # Apply runtime overrides.
    if total_delay_override is not None:
        total_delay = float(total_delay_override)
    if rotation_degrees is not None:
        frame_rotation = float(rotation_degrees)
    if scaling_factor is not None:
        frame_scaling = float(scaling_factor)
    if right_aspect is not None:
        right_canvas_aspect = float(right_aspect)
    if goal_reached_topic_name is not None:
        globals()["goal_reached_topic"] = goal_reached_topic_name
    if is_collision_topic_name is not None:
        globals()["is_collision_topic"] = is_collision_topic_name

    # Sanity checks on user-facing runtime configuration.
    assert frame_scaling > 0.0, "frame_scaling must be > 0"
    assert isinstance(frame_rotation, (int, float)), "frame_rotation must be numeric"
    assert right_canvas_aspect > 0.0, "right_canvas_aspect must be > 0"
    assert panel_layout in {"easy", "medium", "hard"}, "panel_layout must be easy/medium/hard"

    # Setup ROS subscribers if requested and available.
    if use_ros and ROS_AVAILABLE:
        rospy.init_node("cam_client_display_fast", anonymous=True, disable_signals=True)
        rospy.Subscriber(goal_reached_topic, Bool, _on_goal_msg, queue_size=1)
        rospy.Subscriber(is_collision_topic, Bool, _on_collision_msg, queue_size=1)
        print(f"[CLIENT] Subscribed to ROS topics: {goal_reached_topic}, {is_collision_topic}")
    elif use_ros and not ROS_AVAILABLE:
        print("[CLIENT] WARNING: rospy not available. Running without ROS overlays.")

    print(f"[CLIENT] Loading map panel images...")
    images = load_map_images()

    print(f"[CLIENT] Connecting to {server_host}:{server_port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_host, server_port))
    print("[CLIENT] Connected")

    latest_store = LatestPacketStore()
    stop_event = threading.Event()
    recv_thread = threading.Thread(
        target=receiver_thread_main,
        args=(sock, latest_store, stop_event),
        name="camera-receiver",
        daemon=True,
    )
    recv_thread.start()

    # Select decode strategy based on the requested scale.
    decode_flag, decode_scale = maybe_decode_reduced_jpeg(frame_scaling)
    residual_scale = frame_scaling / decode_scale
    assert residual_scale > 0.0, "Residual scale must stay positive"

    clock_offset: Optional[float] = None
    last_seq = 0

    # These caches are keyed by frame shape. They are recreated only when needed.
    solid_tint_cache: Dict[Tuple[int, int, Tuple[int, int, int]], np.ndarray] = {}
    panel_cache: Dict[Tuple[int, int, str], np.ndarray] = {}
    combined_canvas: Optional[np.ndarray] = None
    combined_shape: Optional[Tuple[int, int, int]] = None

    try:
        while True:
            # Wait briefly for a newer packet. The short timeout keeps the UI responsive.
            packet = latest_store.wait_for_newer(last_seq, timeout_sec=0.005)
            if packet is None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("[CLIENT] 'q' pressed, exiting")
                    return
                if latest_store.closed and not recv_thread.is_alive():
                    raise RuntimeError("Receiver thread exited and no newer packet is available")
                continue

            last_seq = packet.seq

            # Decode only the latest JPEG. All stale JPEGs were skipped before decode.
            encoded_np = np.frombuffer(packet.jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(encoded_np, decode_flag)
            if frame is None:
                print("[CLIENT] Failed to decode frame; skipping")
                continue

            # If we used reduced JPEG decode, apply only the remaining scale factor.
            if abs(residual_scale - 1.0) > 1e-9:
                frame = scale_frame(frame, residual_scale)

            # Match the original order: rotate, then scale. When reduced decode is used,
            # the scaling work above has already been applied in the compressed domain.
            display_frame = rotate_frame_cached(frame, frame_rotation)

            # The first successfully displayed packet establishes the server/client clock offset.
            if clock_offset is None:
                clock_offset = packet.arrival_ts - packet.server_ts
                print(f"[CLIENT] Estimated clock offset: {clock_offset:.4f} s")

            ts_client = packet.server_ts + clock_offset # recieved time in client clock



            goal_flag, collision_flag = get_flags()
            if collision_flag:
                apply_overlay_tint_inplace(display_frame, COLLISION_COLOR, OVERLAY_ALPHA, solid_tint_cache)
            elif goal_flag:
                apply_overlay_tint_inplace(display_frame, GOAL_COLOR, OVERLAY_ALPHA, solid_tint_cache)


            display_height, display_width = display_frame.shape[:2]
            panel_cache_key = (display_height, int(round(right_canvas_aspect * 1000)), panel_layout)
            panel = panel_cache.get(panel_cache_key)
            if panel is None:
                panel = build_static_right_panel(
                    images[panel_layout],
                    display_height,
                    aspect=right_canvas_aspect,
                    padding_px=PANEL_PADDING_PX,
                )
                panel_cache[panel_cache_key] = panel

            # Reuse a preallocated combined output buffer whenever the display size is unchanged.
            expected_shape = (display_height, display_width + panel.shape[1], 3)
            if combined_canvas is None or combined_shape != expected_shape:
                combined_canvas = np.empty(expected_shape, dtype=np.uint8)
                combined_shape = expected_shape

            assert combined_canvas is not None
            combined_canvas[:, :display_width] = display_frame
            combined_canvas[:, display_width:] = panel

            if total_delay > 0.01:
                frame_buffer.append((ts_client, combined_canvas.copy()))
                delayed_canvas, frame_buffer = delayed_frame_render(time.time(), frame_buffer)
            else:
                cv2.imshow("Camera Feed", combined_canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[CLIENT] 'q' pressed, exiting")
                return

    except KeyboardInterrupt:
        print("[CLIENT] KeyboardInterrupt - shutting down")
    finally:
        stop_event.set()
        try:
            sock.close()
        except Exception:
            pass
        try:
            recv_thread.join(timeout=0.2)
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("[CLIENT] Shutdown complete")


# -------------------------
# Headless unit tests
# -------------------------
def _unit_tests_quick():# -> None:
    """Basic headless sanity tests for the optimized hot-path helpers."""
    print("[TEST] Running quick unit tests...")

    # Synthetic 200x100 test image with deterministic content.
    width, height = 200, 100
    test_img = np.zeros((height, width, 3), dtype=np.uint8)
    test_img[:, :] = (50, 100, 150)
    cv2.putText(test_img, "TEST", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

    # Rotation preserves the original output dimensions.
    rotated = rotate_frame_cached(test_img, 45.0)
    assert rotated.shape == test_img.shape, "rotate_frame_cached should preserve shape"

    # Scaling changes both dimensions as expected.
    scaled = scale_frame(test_img, 0.5)
    assert scaled.shape == (50, 100, 3), f"Unexpected scaled shape: {scaled.shape}"

    # Reduced JPEG decode selection should map exact scales to OpenCV constants.
    decode_flag, decode_scale = maybe_decode_reduced_jpeg(0.5)
    assert decode_flag == cv2.IMREAD_REDUCED_COLOR_2, "0.5 scale should use reduced JPEG decode"
    assert abs(decode_scale - 0.5) < 1e-9, "Unexpected reduced decode scale"

    # Static panel build should honor the requested output height.
    synthetic_panel = np.zeros((300, 500, 4), dtype=np.uint8)
    synthetic_panel[:, :, :3] = (10, 20, 30)
    synthetic_panel[:, :, 3] = 200
    panel = build_static_right_panel(synthetic_panel, 240, aspect=1.5, padding_px=12)
    assert panel.shape[0] == 240, "Panel height mismatch"
    assert panel.shape[2] == 3, "Panel must be BGR"

    # Latest-packet store should overwrite stale packets.
    store = LatestPacketStore()
    store.put(server_ts=1.0, arrival_ts=2.0, jpg_bytes=bytearray(b"abc"))
    first = store.wait_for_newer(0, timeout_sec=0.0)
    assert first is not None and first.seq == 1, "Did not retrieve first packet"
    store.put(server_ts=3.0, arrival_ts=4.0, jpg_bytes=bytearray(b"def"))
    second = store.wait_for_newer(1, timeout_sec=0.0)
    assert second is not None and second.seq == 2, "Did not retrieve second packet"

    # In-place tint and timing overlay should preserve shape and dtype.
    work = test_img.copy()
    tint_cache: Dict[Tuple[int, int, Tuple[int, int, int]], np.ndarray] = {}
    apply_overlay_tint_inplace(work, GOAL_COLOR, OVERLAY_ALPHA, tint_cache)
    draw_timing_overlay_inplace(work, 1.0, 2.0, 3.0)
    assert work.shape == test_img.shape, "Overlay path changed image shape"
    assert work.dtype == np.uint8, "Overlay path changed image dtype"

    print("[TEST] Quick unit tests passed.")


# -------------------------
# CLI entrypoint
# -------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Low-latency camera client with cached map panel")
    parser.add_argument("--host", default="192.168.0.172", help="camera server hostname")
    parser.add_argument("--port", type=int, default=5000, help="camera server port")
    parser.add_argument("--no-ros", action="store_true", help="run without attempting ROS")
    parser.add_argument("--rotation", type=float, default=None, help="frame rotation degrees")
    parser.add_argument("--scaling", type=float, default=None, help="frame scaling factor")
    parser.add_argument("--right-aspect", type=float, default=None, help="right canvas aspect ratio W/H")
    parser.add_argument("--layout", choices=["easy", "medium", "hard"], default=DEFAULT_LAYOUT, help="map panel layout")
    parser.add_argument("--delay", type=float, default=None, help="optional display delay in seconds")
    parser.add_argument("--test", action="store_true", help="run quick self-tests and exit")
    args = parser.parse_args()

    if args.test:
        _unit_tests_quick()
    else:
        start_client(
            server_host=args.host,
            server_port=args.port,
            total_delay_override=args.delay,
            rotation_degrees=args.rotation,
            scaling_factor=args.scaling,
            right_aspect=args.right_aspect,
            panel_layout=args.layout,
            use_ros=(not args.no_ros),
        )
