#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Low-latency camera feed client with ROS topic overlays and a cached right panel.
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

try:
    import rospy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except Exception:
    rospy = None
    Bool = None
    Image = None
    CvBridge = None
    ROS_AVAILABLE = False

# -------------------------
# User-configurable globals
# -------------------------
goal_reached_topic = "/goal_reached"
is_collision_topic = "/safety_status" # Matched to your safety controller

frame_rotation = 270.0
frame_scaling = 0.6
right_canvas_aspect = 3.0 / 2.0
total_delay = 0.001

OVERLAY_ALPHA = 0.35
GOAL_COLOR = (0, 200, 0)
COLLISION_COLOR = (0, 0, 200)

USE_REDUCED_JPEG_DECODE = True
DEFAULT_LAYOUT = "medium"
PANEL_PADDING_PX = 12

_flags_lock = threading.Lock()
_goal_reached_flag = False
_collision_flag = False

def set_goal_flag(value):
    global _goal_reached_flag
    _goal_reached_flag = bool(value)

def set_collision_flag(value):
    global _collision_flag
    _collision_flag = bool(value)

def get_flags():
    return bool(_goal_reached_flag), bool(_collision_flag)

def _on_goal_msg(msg):
    try: set_goal_flag(bool(msg.data))
    except Exception: set_goal_flag(False)

def _on_collision_msg(msg):
    try: set_collision_flag(bool(msg.data))
    except Exception: set_collision_flag(False)

# -------------------------
# Static panel helpers
# -------------------------
def load_map_images(image_dir="/home/fetchuser/chinmay/fetch_teleop_ws/src/fetch_teleop/cam_files/"):
    image_dict = {}
    for layout in ["easy", "medium", "hard", "train"]: # Added 'train' for Figure 8
        path = f"{image_dir}map_{layout}.png"
        cached_panel_image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if cached_panel_image is None:
            print(f"[CLIENT] WARNING: Failed to load panel image: {path} (Will show blank space)")
        image_dict[layout] = cached_panel_image
    return image_dict

def build_static_right_panel(source_image, canvas_height, aspect, padding_px=PANEL_PADDING_PX):
    canvas_width = max(1, int(round(canvas_height * aspect)))
    panel = np.full((canvas_height, canvas_width, 3), 240, dtype=np.uint8)
    cv2.rectangle(panel, (0, 0), (canvas_width - 1, canvas_height - 1), (200, 200, 200), 1)

    if source_image is None:
        return panel

    inner_height = canvas_height - 2 * padding_px
    inner_width = canvas_width - 2 * padding_px

    if source_image.shape[2] == 4:
        src_bgr = source_image[:, :, :3]
        src_alpha = source_image[:, :, 3]
    else:
        src_bgr = source_image
        src_alpha = None

    src_height, src_width = src_bgr.shape[:2]
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

    if src_alpha is None:
        roi[:] = resized_bgr
    else:
        resized_alpha = cv2.resize(src_alpha, (new_width, new_height), interpolation=interpolation)
        alpha_f = (resized_alpha.astype(np.float32) / 255.0)[:, :, None]
        src_f = resized_bgr.astype(np.float32)
        dst_f = roi.astype(np.float32)
        roi[:] = np.clip(src_f * alpha_f + dst_f * (1.0 - alpha_f), 0.0, 255.0).astype(np.uint8)

    return panel

_rotation_matrix_cache = {}

def rotate_frame_cached(frame, degrees):
    if degrees % 360 == 0:
        return frame
    height, width = frame.shape[:2]
    cache_key = (height, width, float(degrees))
    M = _rotation_matrix_cache.get(cache_key)
    if M is None:
        center = (width / 2.0, height / 2.0)
        M = cv2.getRotationMatrix2D(center, degrees, 1.0)
        _rotation_matrix_cache[cache_key] = M
    return cv2.warpAffine(frame, M, (width, height), flags=cv2.INTER_LINEAR)

def scale_frame(frame, scale):
    if abs(scale - 1.0) < 1e-9:
        return frame
    height, width = frame.shape[:2]
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(frame, (new_width, new_height), interpolation=interpolation)

def maybe_decode_reduced_jpeg(scale):
    if not USE_REDUCED_JPEG_DECODE: return cv2.IMREAD_COLOR, 1.0
    if abs(scale - 0.5) < 1e-9: return cv2.IMREAD_REDUCED_COLOR_2, 0.5
    if abs(scale - 0.25) < 1e-9: return cv2.IMREAD_REDUCED_COLOR_4, 0.25
    if abs(scale - 0.125) < 1e-9: return cv2.IMREAD_REDUCED_COLOR_8, 0.125
    return cv2.IMREAD_COLOR, 1.0

def apply_overlay_tint_inplace(frame, color, alpha, solid_cache):
    cache_key = (frame.shape[0], frame.shape[1], color)
    solid = solid_cache.get(cache_key)
    if solid is None:
        solid = np.empty_like(frame)
        solid[:] = color
        solid_cache[cache_key] = solid
    cv2.addWeighted(solid, alpha, frame, 1.0 - alpha, 0.0, frame)

def darken_text_background_inplace(frame, x0, y0, width, height, alpha):
    x1, y1 = max(0, x0), max(0, y0)
    x2, y2 = min(frame.shape[1], x0 + width), min(frame.shape[0], y0 + height)
    if x2 <= x1 or y2 <= y1: return
    roi = frame[y1:y2, x1:x2]
    cv2.convertScaleAbs(roi, dst=roi, alpha=(1.0 - alpha), beta=0.0)

def delayed_frame_render(now, frame_buffer):
    last_rem_time = 1
    for i, frame_item in enumerate(frame_buffer):
        frame_ts, frame = frame_item
        rem_time = total_delay - (now - frame_ts)
        if rem_time > 0 and last_rem_time < 0:
            render_frame = frame_buffer[i-1][1]
            return render_frame, frame_buffer[i:]
        last_rem_time = rem_time
    return None, frame_buffer

def draw_timing_overlay_inplace(frame, net_ms, added_ms, total_ms):
    text_lines = [
        f"Network: {net_ms:7.1f} ms",
        f"Added:   {added_ms:7.1f} ms",
        f"Total:   {total_ms:7.1f} ms",
    ]
    x, y0, dy = 10, 30, 25
    darken_text_background_inplace(frame, x0=x-5, y0=5, width=260, height=dy * len(text_lines) + 10, alpha=0.4)
    for i, line in enumerate(text_lines):
        cv2.putText(frame, line, (x, y0 + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

def recv_exact(sock, count):
    buf = bytearray(count)
    view = memoryview(buf)
    received = 0
    while received < count:
        nbytes = sock.recv_into(view[received:], count - received)
        if nbytes == 0: raise ValueError("Socket closed while receiving data")
        received += nbytes
    return buf

@dataclass
class LatestJpegPacket:
    seq: int
    server_ts: float
    arrival_ts: float
    jpg_bytes: bytearray

class LatestPacketStore:
    def __init__(self):
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0
        self._closed = False
        self._error = None

    def put(self, server_ts, arrival_ts, jpg_bytes):
        with self._cond:
            self._seq += 1
            self._latest = LatestJpegPacket(seq=self._seq, server_ts=float(server_ts), arrival_ts=float(arrival_ts), jpg_bytes=jpg_bytes)
            self._cond.notify_all()

    def set_closed(self, error=None):
        with self._cond:
            self._closed = True
            self._error = error
            self._cond.notify_all()

    def wait_for_newer(self, last_seq, timeout_sec=0.01):
        with self._cond:
            if self._latest is None or self._latest.seq <= last_seq:
                self._cond.wait(timeout=timeout_sec)
            if self._latest is not None and self._latest.seq > last_seq:
                return self._latest
            if self._closed and self._error is not None:
                raise RuntimeError(f"Receiver thread failed: {self._error}") from self._error
            return None

    @property
    def closed(self):
        with self._cond: return self._closed

def receiver_thread_main(sock, latest_store, stop_event):
    try:
        while not stop_event.is_set():
            header = recv_exact(sock, 8 + 4)
            server_ts, frame_size = struct.unpack("!dI", header)
            if frame_size <= 0: raise ValueError(f"Received invalid frame size: {frame_size}")
            jpg_bytes = recv_exact(sock, frame_size)
            arrival_ts = time.time()
            latest_store.put(server_ts, arrival_ts, jpg_bytes)
    except Exception as exc:
        latest_store.set_closed(error=exc)

# -------------------------
# Main client
# -------------------------
def start_client(server_host, server_port, *, total_delay_override=None, rotation_degrees=None, scaling_factor=None, right_aspect=None, panel_layout=DEFAULT_LAYOUT, use_ros=True):
    global total_delay, frame_rotation, frame_scaling, right_canvas_aspect

    image_pub = None
    bridge = None

    if use_ros and ROS_AVAILABLE:
        rospy.init_node("cam_client_display_fast", anonymous=True, disable_signals=True)
        rospy.Subscriber(goal_reached_topic, Bool, _on_goal_msg, queue_size=1)
        rospy.Subscriber(is_collision_topic, Bool, _on_collision_msg, queue_size=1)
        
        # --- READ ROS PARAMS ---
        # Overrides argparse if run via roslaunch
        server_host = rospy.get_param("~server_host", server_host)
        server_port = rospy.get_param("~server_port", server_port)
        total_delay_override = rospy.get_param("~delay", total_delay_override)
        panel_layout = rospy.get_param("~layout", panel_layout)

        image_pub = rospy.Publisher('/external_camera/image_delayed', Image, queue_size=1)
        bridge = CvBridge()
        print(f"[CLIENT] Subscribed to ROS topics. Layout param: {panel_layout}")

    # Apply runtime overrides
    if total_delay_override is not None: total_delay = float(total_delay_override)
    if rotation_degrees is not None: frame_rotation = float(rotation_degrees)
    if scaling_factor is not None: frame_scaling = float(scaling_factor)
    if right_aspect is not None: right_canvas_aspect = float(right_aspect)

    print(f"[CLIENT] Loading map panel images...")
    images = load_map_images()

    print(f"[CLIENT] Connecting to {server_host}:{server_port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_host, server_port))
    print("[CLIENT] Connected")

    latest_store = LatestPacketStore()
    stop_event = threading.Event()
    recv_thread = threading.Thread(target=receiver_thread_main, args=(sock, latest_store, stop_event), daemon=True)
    recv_thread.start()

    decode_flag, decode_scale = maybe_decode_reduced_jpeg(frame_scaling)
    residual_scale = frame_scaling / decode_scale

    clock_offset = None
    last_seq = 0

    solid_tint_cache = {}
    panel_cache = {}
    combined_canvas = None
    combined_shape = None
    frame_buffer = []

    try:
        while True:
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
            encoded_np = np.frombuffer(packet.jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(encoded_np, decode_flag)
            if frame is None: continue

            if abs(residual_scale - 1.0) > 1e-9:
                frame = scale_frame(frame, residual_scale)

            display_frame = rotate_frame_cached(frame, frame_rotation)

            if clock_offset is None:
                clock_offset = packet.arrival_ts - packet.server_ts

            ts_client = packet.server_ts + clock_offset 
            now = time.time()

            # --- CALCULATE LATENCY ---
            network_delay = max(packet.arrival_ts - ts_client, 0.0)
            total_delay_measured = max(now - ts_client, 0.0)
            added_delay = max(total_delay_measured - network_delay, 0.0)

            # --- APPLY TINTS ---
            goal_flag, collision_flag = get_flags()
            if collision_flag:
                apply_overlay_tint_inplace(display_frame, COLLISION_COLOR, OVERLAY_ALPHA, solid_tint_cache)
            elif goal_flag:
                apply_overlay_tint_inplace(display_frame, GOAL_COLOR, OVERLAY_ALPHA, solid_tint_cache)

            # --- FIX: CALL THE TIMING TEXT FUNCTION ---
            draw_timing_overlay_inplace(display_frame, network_delay * 1000.0, added_delay * 1000.0, total_delay_measured * 1000.0)

            display_height, display_width = display_frame.shape[:2]
            
            # --- RENDER RIGHT PANEL ---
            layout_key = panel_layout.lower()
            panel_cache_key = (display_height, int(round(right_canvas_aspect * 1000)), layout_key)
            panel = panel_cache.get(panel_cache_key)
            if panel is None:
                panel = build_static_right_panel(
                    images.get(layout_key), # Pulls correctly from dict
                    display_height,
                    aspect=right_canvas_aspect,
                    padding_px=PANEL_PADDING_PX,
                )
                panel_cache[panel_cache_key] = panel

            expected_shape = (display_height, display_width + panel.shape[1], 3)
            if combined_canvas is None or combined_shape != expected_shape:
                combined_canvas = np.empty(expected_shape, dtype=np.uint8)
                combined_shape = expected_shape

            combined_canvas[:, :display_width] = display_frame
            combined_canvas[:, display_width:] = panel

            # --- DELAY BUFFER LOGIC ---
            if total_delay > 0.01:
                frame_buffer.append((ts_client, combined_canvas.copy()))
                delayed_canvas, frame_buffer = delayed_frame_render(now, frame_buffer)
                if delayed_canvas is not None:
                    cv2.imshow("Participant Feed", delayed_canvas)
                    if image_pub:
                        try: image_pub.publish(bridge.cv2_to_imgmsg(delayed_canvas, "bgr8"))
                        except Exception: pass
            else:
                cv2.imshow("Participant Feed", combined_canvas)
                if image_pub:
                    try: image_pub.publish(bridge.cv2_to_imgmsg(combined_canvas, "bgr8"))
                    except Exception: pass
            
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return

    except KeyboardInterrupt:
        print("[CLIENT] KeyboardInterrupt - shutting down")
    finally:
        stop_event.set()
        try: sock.close()
        except Exception: pass
        try: recv_thread.join(timeout=0.2)
        except Exception: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.172")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--layout", choices=["easy", "medium", "hard", "train"], default=DEFAULT_LAYOUT)
    parser.add_argument("--delay", type=float, default=None)
    args = parser.parse_args()

    start_client(server_host=args.host, server_port=args.port, total_delay_override=args.delay, panel_layout=args.layout, use_ros=(not args.no_ros))