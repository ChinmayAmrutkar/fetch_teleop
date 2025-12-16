#!/usr/bin/env python3
import socket
import struct
import time

import cv2
import numpy as np


def start_server(
    host="0.0.0.0",
    port=5000,
    camera_index=0,
    width=640,
    height=480,
    fps=30,
    jpeg_quality=80,
):
    """
    Stream frames from a USB camera over TCP with timestamps.

    Each frame is sent as:
        [8-byte double: server_timestamp][4-byte uint: frame_size][frame_bytes]
    """
    # Open camera
    cap = cv2.VideoCapture(camera_index) #  Use V4L2 backend for better compatibility on Linux
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")
    # try:
    #     cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2) #  Use V4L2 backend for better compatibility on Linux
    #     if not cap.isOpened():
    #         raise RuntimeError(f"Could not open camera index {camera_index}")
    # except Exception as e:
    #     print(f"[SERVER] Error opening camera: {e}")
    #     cap = cv2.VideoCapture(camera_index)
    #     if not cap.isOpened():
    #         raise RuntimeError(f"Could not open camera index {camera_index}")

    # Set basic properties (best-effort; may not be honored by all drivers)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # TCP server socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Reuse port quickly after restart
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((host, port))
    server_sock.listen(1)

    print(f"[SERVER] Listening on {host}:{port}...")
    conn, addr = server_sock.accept()
    print(f"[SERVER] Client connected from {addr}")

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[SERVER] Failed to grab frame")
                break

            cv2.imshow("server preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Optional: convert color if needed
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Timestamp in seconds (float)
            ts = time.time()

            # Encode as JPEG to keep bandwidth low and decoding simple
            success, encoded = cv2.imencode(".jpg", frame, encode_params)
            if not success:
                print("[SERVER] JPEG encoding failed")
                continue

            data = encoded.tobytes()
            frame_size = len(data)

            # Pack header: 8-byte double (timestamp) + 4-byte uint (size)
            header = struct.pack("!dI", ts, frame_size)

            # Send header + frame
            try:
                conn.sendall(header + data)
            except (BrokenPipeError, ConnectionResetError):
                print("[SERVER] Connection lost")
                break

    finally:
        cap.release()
        conn.close()
        server_sock.close()
        print("[SERVER] Shutdown complete")


if __name__ == "__main__":
    # Adjust host/port/camera_index as needed
    start_server(
        host="0.0.0.0",
        port=5000,
        camera_index=0,
        width=640,
        height=480,
        fps=30,
        jpeg_quality=80,
    )
    # start_server(
    #     host="127.0.0.1",
    #     port=5000,
    #     camera_index=0,
    #     width=640,
    #     height=480,
    #     fps=30,
    #     jpeg_quality=80,
    # )