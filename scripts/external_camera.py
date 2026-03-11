#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Delayed camera feed client with ROS topic overlays and frame transforms.
Features: Tints for Goal/Collision, Configurable Rotation/Scaling, and a Side Canvas for Maps.
"""
import socket
import struct
import time
from collections import deque
import threading
import argparse

import cv2
import numpy as np

# ROS Imports
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# -------------------------
# User-configurable globals
# -------------------------
goal_reached_topic = "/goal_reached"
is_collision_topic = "/safety_status"

frame_rotation = 90.0  # degrees
frame_scaling = 1.0  # 1.0 = original size
right_canvas_aspect = 3.0 / 2.0
OVERLAY_ALPHA = 0.35

GOAL_COLOR = (0, 200, 0)      # green
COLLISION_COLOR = (0, 0, 200) # red

# -------------------------
# Thread-safe flags
# -------------------------
_flags_lock = threading.Lock()
_goal_reached_flag = False
_collision_flag = False

def set_goal_flag(value):
    global _goal_reached_flag
    with _flags_lock:
        _goal_reached_flag = bool(value)

def set_collision_flag(value):
    global _collision_flag
    with _flags_lock:
        _collision_flag = bool(value)

def get_flags():
    with _flags_lock:
        return bool(_goal_reached_flag), bool(_collision_flag)

# -------------------------
# ROS callbacks
# -------------------------
def _on_goal_msg(msg):
    try: set_goal_flag(bool(msg.data))
    except Exception: set_goal_flag(False)

def _on_collision_msg(msg):
    try: set_collision_flag(bool(msg.data))
    except Exception: set_collision_flag(False)

# -------------------------
# Image transform helpers
# -------------------------
def rotate_frame(frame, degrees):
    if degrees % 360 == 0:
        return frame
    (h, w) = frame.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, degrees, 1.0)
    return cv2.warpAffine(frame, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

def scale_frame(frame, scale):
    if abs(scale - 1.0) < 1e-9:
        return frame
    h, w = frame.shape[:2]
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

def make_right_canvas_for_height(height, aspect=right_canvas_aspect):
    canvas_w = max(1, int(round(height * aspect)))
    canvas = np.full((height, canvas_w, 3), 240, dtype=np.uint8)
    cv2.rectangle(canvas, (0, 0), (canvas_w - 1, height - 1), (200, 200, 200), 1)
    return canvas

def apply_overlay_tint(frame, color, alpha):
    overlay = frame.copy()
    h, w = frame.shape[:2]
    cv2.rectangle(overlay, (0, 0), (w, h), color, thickness=-1)
    return cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)

def recv_exact(sock, count):
    buf = b""
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk: raise ValueError("Socket closed while receiving data")
        buf += chunk
    return buf

def load_image_onto_right_canvas(canvas_bgr, image, fit="contain", padding_px=10, allow_upscale=True, draw_border=False, border_color_bgr=(200, 200, 200), border_thickness=1):
    canvas_h, canvas_w = canvas_bgr.shape[:2]
    inner_h = canvas_h - 2 * padding_px
    inner_w = canvas_w - 2 * padding_px
    out = canvas_bgr.copy()

    if isinstance(image, str):
        src = cv2.imread(image, cv2.IMREAD_UNCHANGED)
        if src is None: return out
    else:
        src = image

    if src.ndim == 2: src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    elif src.ndim == 3 and src.shape[2] == 1: src = cv2.cvtColor(src[:, :, 0], cv2.COLOR_GRAY2BGR)

    if src.ndim == 3 and src.shape[2] == 4:
        src_bgr = src[:, :, :3]
        src_alpha = src[:, :, 3]
    else:
        src_bgr = src
        src_alpha = None

    src_h, src_w = src_bgr.shape[:2]
    scale_x = inner_w / float(src_w)
    scale_y = inner_h / float(src_h)

    scale = min(scale_x, scale_y) if fit == "contain" else max(scale_x, scale_y)
    if not allow_upscale: scale = min(scale, 1.0)
    
    new_w = max(1, int(round(src_w * scale)))
    new_h = max(1, int(round(src_h * scale)))
    interp = cv2.INTER_AREA if (new_w < src_w or new_h < src_h) else cv2.INTER_LINEAR
    
    resized_bgr = cv2.resize(src_bgr, (new_w, new_h), interpolation=interp)
    resized_alpha = None if src_alpha is None else cv2.resize(src_alpha, (new_w, new_h), interpolation=interp)

    place_h, place_w = resized_bgr.shape[:2]
    dst_x0 = padding_px + max(0, (inner_w - place_w) // 2)
    dst_y0 = padding_px + max(0, (inner_h - place_h) // 2)
    dst_x1 = dst_x0 + place_w
    dst_y1 = dst_y0 + place_h

    roi = out[dst_y0:dst_y1, dst_x0:dst_x1]
    
    if resized_alpha is None:
        out[dst_y0:dst_y1, dst_x0:dst_x1] = resized_bgr
    else:
        alpha_f = (resized_alpha.astype(np.float32) / 255.0)[:, :, None]
        blended = resized_bgr.astype(np.float32) * alpha_f + roi.astype(np.float32) * (1.0 - alpha_f)
        out[dst_y0:dst_y1, dst_x0:dst_x1] = np.clip(blended, 0.0, 255.0).astype(np.uint8)

    if draw_border and border_thickness > 0:
        cv2.rectangle(out, (dst_x0, dst_y0), (dst_x1 - 1, dst_y1 - 1), border_color_bgr, int(border_thickness))
    return out

# -------------------------
# Main client
# -------------------------
def start_client():
    rospy.init_node("external_camera_node", anonymous=True)

    # ROS Parameters
    server_host = rospy.get_param("~server_host", "192.168.0.172")
    server_port = rospy.get_param("~server_port", 5000)
    total_delay = rospy.get_param("~delay", 1.0)
    map_image_path = rospy.get_param("~map_image_path", "") # Pass map path via launch file

    # Subscribers & Publishers
    rospy.Subscriber(goal_reached_topic, Bool, _on_goal_msg, queue_size=1)
    rospy.Subscriber(is_collision_topic, Bool, _on_collision_msg, queue_size=1)
    image_pub = rospy.Publisher('/external_camera/image_delayed', Image, queue_size=10)
    bridge = CvBridge()

    # Load Map Image (Once, outside the loop)
    cached_panel_image = None
    if map_image_path:
        cached_panel_image = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if cached_panel_image is None:
            rospy.logwarn("[CLIENT] Failed to load panel map image from: {}".format(map_image_path))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("[CLIENT] Connecting to {}:{} ...".format(server_host, server_port))
    sock.connect((server_host, server_port))
    print("[CLIENT] Connected. Delay target: {}s".format(total_delay))

    frame_buffer = deque()
    clock_offset = None

    try:
        while not rospy.is_shutdown():
            try:
                header = recv_exact(sock, 8 + 4)
            except ValueError:
                break
            
            server_ts, frame_size = struct.unpack("!dI", header)
            jpg_bytes = recv_exact(sock, frame_size)
            arrival_time_client = time.time()

            nparr = np.frombuffer(jpg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None: continue

            if clock_offset is None:
                clock_offset = arrival_time_client - server_ts

            ts_client = server_ts + clock_offset
            target_display_time = ts_client + total_delay
            frame_buffer.append((server_ts, arrival_time_client, target_display_time, frame))

            while frame_buffer:
                server_ts0, arrival0, t_display0, fr = frame_buffer[0]
                now = time.time()
                wait = t_display0 - now
                
                if wait > 0:
                    time.sleep(min(wait, 0.005))
                    break

                frame_buffer.popleft()
                ts_client0 = server_ts0 + clock_offset

                net_ms = max(arrival0 - ts_client0, 0.0) * 1000.0
                added_ms = max(now - arrival0, 0.0) * 1000.0
                total_ms = max(now - ts_client0, 0.0) * 1000.0

                # 1. Transform
                display_frame = rotate_frame(fr, frame_rotation)
                display_frame = scale_frame(display_frame, frame_scaling)

                # 2. Tints
                goal_flag, collision_flag = get_flags()
                if collision_flag:
                    display_with_tint = apply_overlay_tint(display_frame, COLLISION_COLOR, OVERLAY_ALPHA)
                elif goal_flag:
                    display_with_tint = apply_overlay_tint(display_frame, GOAL_COLOR, OVERLAY_ALPHA)
                else:
                    display_with_tint = display_frame

                # 3. Text Overlay
                overlay_for_text = display_with_tint.copy()
                text_lines = [
                    "Network: {:7.1f} ms".format(net_ms),
                    "Added:   {:7.1f} ms".format(added_ms),
                    "Total:   {:7.1f} ms".format(total_ms)
                ]
                
                # Dark background for text
                cv2.rectangle(overlay_for_text, (5, 5), (260, 85), (0, 0, 0), thickness=-1)
                cv2.addWeighted(overlay_for_text, 0.4, display_with_tint, 0.6, 0, overlay_for_text)

                for i, line in enumerate(text_lines):
                    cv2.putText(overlay_for_text, line, (10, 30 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                # 4. Right Canvas Map logic
                h_disp = overlay_for_text.shape[0]
                canvas = make_right_canvas_for_height(h_disp, aspect=right_canvas_aspect)
                if cached_panel_image is not None:
                    canvas = load_image_onto_right_canvas(canvas, cached_panel_image, fit="contain", padding_px=12)
                
                # Combine
                combined = np.hstack((overlay_for_text, canvas))

                # 5. Show & Publish
                try:
                    ros_msg = bridge.cv2_to_imgmsg(combined, encoding="bgr8")
                    image_pub.publish(ros_msg)
                except Exception as e:
                    pass

                cv2.imshow("Participant Interface", combined)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    rospy.signal_shutdown("Quit")
                    return

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    start_client()


##roslaunch fetch_teleop safety.launch method:=delayed delay_time:=1.0 external_camera/map_image_path:="/home/fetchuser/my_maps/map_1.png"