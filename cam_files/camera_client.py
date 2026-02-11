#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Delayed camera feed client with ROS topic overlays and frame transforms.

Features:
1. Subscribes to a ROS Bool topic (goal_reached_topic) and overlays a
   green semi-transparent tint when True.
2. Subscribes to a ROS Bool topic (is_collision_topic) and overlays a
   red semi-transparent tint when True. If both are True, red wins.
3. frame_rotation (degrees) rotates displayed frames by the requested angle.
4. frame_scaling rescales displayed frames by the requested factor.
5. Adds a blank canvas to the right of the camera feed with a 3:2 (W:H)
   aspect ratio and height matched to the displayed frame height.
"""
import socket
import struct
import time
from collections import deque
import threading
import argparse

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
# Topic names to listen to (strings). If running without ROS, these are ignored.
goal_reached_topic = "/goal_reached"
is_collision_topic = "/safety_status"

# How many degrees to rotate displayed frames (clockwise positive).
# frame_rotation = 0.0  # degrees
frame_rotation = 90.0  # degrees

# Scaling factor for displayed frames. Must be > 0.
frame_scaling = 2.0  # 1.0 = original size; 0.5 = half size; 2.0 = double size

# Canvas aspect ratio to the right (width : height). For 3:2 set 3/2.
right_canvas_aspect = 3.0 / 2.0

# Desired total delay (sec) between capture and display (used by timing overlay).
total_delay = 0.001
estimated_base_latency = 0.100

# Overlay alpha for semi-transparency
OVERLAY_ALPHA = 0.35  # 0 = fully transparent, 1 = fully opaque

# Color definitions (BGR)
GOAL_COLOR = (0, 200, 0)      # green
COLLISION_COLOR = (0, 0, 200) # red

# -------------------------
# Thread-safe flags for ROS callbacks
# -------------------------
_flags_lock = threading.Lock()
_goal_reached_flag = False
_collision_flag = False

def set_goal_flag(value: bool):
    """Thread-safe setter for goal flag (internal)."""
    global _goal_reached_flag
    with _flags_lock:
        _goal_reached_flag = bool(value)

def set_collision_flag(value: bool):
    """Thread-safe setter for collision flag (internal)."""
    global _collision_flag
    with _flags_lock:
        _collision_flag = bool(value)

def get_flags():
    """Thread-safe getter returning (goal_reached, collision)."""
    with _flags_lock:
        return bool(_goal_reached_flag), bool(_collision_flag)

# -------------------------
# ROS callbacks (if available)
# -------------------------
def _on_goal_msg(msg):
    """Callback for goal_reached_topic (expects std_msgs/Bool)."""
    try:
        set_goal_flag(bool(msg.data))
    except Exception:
        # defensive: if message doesn't have .data, ignore
        set_goal_flag(False)

def _on_collision_msg(msg):
    """Callback for is_collision_topic (expects std_msgs/Bool)."""
    try:
        set_collision_flag(bool(msg.data))
    except Exception:
        set_collision_flag(False)

# -------------------------
# Image transform helpers
# -------------------------
def rotate_frame(frame: np.ndarray, degrees: float) -> np.ndarray:
    """
    Rotate an image about its center by `degrees`.
    Keeps the same output size (may crop corners).
    """
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    if degrees % 360 == 0:
        return frame
    (h, w) = frame.shape[:2]
    center = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(center, degrees, 1.0)
    rotated = cv2.warpAffine(frame, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return rotated

def scale_frame(frame: np.ndarray, scale: float) -> np.ndarray:
    """
    Scale an image by `scale`. scale must be positive.
    """
    assert isinstance(frame, np.ndarray), "frame must be a numpy array"
    assert scale > 0.0, "scale must be > 0"
    if abs(scale - 1.0) < 1e-9:
        return frame
    h, w = frame.shape[:2]
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

def make_right_canvas_for_height(height: int, aspect: float = right_canvas_aspect) -> np.ndarray:
    """
    Create a blank (white) canvas that matches the given height and has the
    specified aspect ratio width:height. Returns an HxW x3 uint8 array.
    """
    assert height > 0, "height must be positive"
    assert aspect > 0, "aspect must be positive"
    canvas_w = max(1, int(round(height * aspect)))
    canvas = np.full((height, canvas_w, 3), 240, dtype=np.uint8)  # light gray
    # Optional: draw a subtle border
    cv2.rectangle(canvas, (0, 0), (canvas_w - 1, height - 1), (200, 200, 200), 1)
    return canvas

def apply_overlay_tint(frame: np.ndarray, color: tuple, alpha: float) -> np.ndarray:
    """
    Apply a semi-transparent color overlay across the whole frame.
    - color is BGR tuple of ints (0-255)
    - alpha is overlay weight [0..1]
    Returns new image (does not modify input).
    """
    assert 0.0 <= alpha <= 1.0, "alpha must be between 0 and 1"
    overlay = frame.copy()
    h, w = frame.shape[:2]
    # Fill overlay
    cv2.rectangle(overlay, (0, 0), (w, h), color, thickness=-1)
    # Blend
    blended = cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)
    return blended

# -------------------------
# Networking helper
# -------------------------
def recv_exact(sock: socket.socket, count: int) -> bytes:
    """Receive exactly `count` bytes from the socket, raise on EOF."""
    buf = b""
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk:
            raise ValueError("Socket closed while receiving data")
        buf += chunk
    return buf

# -------------------------
# Main client
# -------------------------
def start_client(
    server_host="192.168.0.172",
    server_port=5000,
    *,
    total_delay_override=None,
    goal_reached_topic_name=None,
    is_collision_topic_name=None,
    rotation_degrees=None,
    scaling_factor=None,
    right_aspect=None,
    use_ros=True,
):
    """
    Connect to the server, receive timestamped JPEG frames and display them
    with timing overlays and ROS-driven semi-transparent tints.

    Parameters can override module-level globals; None means "use global".
    """
    global total_delay, frame_rotation, frame_scaling, right_canvas_aspect

    # Apply overrides (if any)
    if total_delay_override is not None:
        total_delay = float(total_delay_override)
    if rotation_degrees is not None:
        frame_rotation = float(rotation_degrees)
    if scaling_factor is not None:
        frame_scaling = float(scaling_factor)
    if right_aspect is not None:
        right_canvas_aspect = float(right_aspect)
    if goal_reached_topic_name is not None:
        # override global var
        globals()['goal_reached_topic'] = goal_reached_topic_name
    if is_collision_topic_name is not None:
        globals()['is_collision_topic'] = is_collision_topic_name

    # sanity checks
    assert frame_scaling > 0.0, "frame_scaling must be > 0"
    assert isinstance(frame_rotation, (int, float)), "frame_rotation must be numeric"
    assert right_canvas_aspect > 0.0, "right_canvas_aspect must be > 0"

    # Setup ROS subscribers if requested and available
    if use_ros and ROS_AVAILABLE:
        # disable_signals to avoid ROS intercepting SIGINT (we handle KeyboardInterrupt)
        rospy.init_node("cam_client_display", anonymous=True, disable_signals=True)
        rospy.Subscriber(goal_reached_topic, Bool, _on_goal_msg, queue_size=1)
        rospy.Subscriber(is_collision_topic, Bool, _on_collision_msg, queue_size=1)
        print(f"[CLIENT] Subscribed to ROS topics: {goal_reached_topic}, {is_collision_topic}")
    elif use_ros and not ROS_AVAILABLE:
        print("[CLIENT] WARNING: rospy not available. Running without ROS overlays.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("[CLIENT] Connecting to {}:{} ...".format(server_host, server_port))
    sock.connect((server_host, server_port))
    print("[CLIENT] Connected")

    frame_buffer = deque()
    clock_offset = None

    try:
        while True:
            # Header: 8-byte double (server_ts) + 4-byte uint (frame_size)
            header = recv_exact(sock, 8 + 4)
            server_ts, frame_size = struct.unpack("!dI", header)

            jpg_bytes = recv_exact(sock, frame_size)
            arrival_time_client = time.time()

            # Decode JPEG - defensive check
            nparr = np.frombuffer(jpg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                print("[CLIENT] Failed to decode frame; skipping")
                continue

            # Clock sync on first frame
            if clock_offset is None:
                clock_offset = arrival_time_client - server_ts
                print(f"[CLIENT] Estimated clock offset: {clock_offset:.4f} s")

            ts_client = server_ts + clock_offset
            target_display_time = ts_client + total_delay
            frame_buffer.append((server_ts, arrival_time_client, target_display_time, frame))

            # Display any frames that are due
            while frame_buffer:
                server_ts0, arrival0, t_display0, fr = frame_buffer[0]
                now = time.time()
                wait = t_display0 - now
                if wait > 0:
                    # Not yet time to display this frame
                    time.sleep(min(wait, 0.005))
                    break

                # Pop and show
                frame_buffer.popleft()
                ts_client0 = server_ts0 + clock_offset

                network_delay = max(arrival0 - ts_client0, 0.0)
                total_delay_measured = max(now - ts_client0, 0.0)
                added_delay = max(total_delay_measured - network_delay, 0.0)

                net_ms = network_delay * 1000.0
                added_ms = added_delay * 1000.0
                total_ms = total_delay_measured * 1000.0

                # Apply rotation then scaling
                display_frame = rotate_frame(fr, frame_rotation)
                display_frame = scale_frame(display_frame, frame_scaling)

                # Decide overlay color based on flags (collision has priority)
                goal_flag, collision_flag = get_flags()

                if collision_flag:
                    display_with_tint = apply_overlay_tint(display_frame, COLLISION_COLOR, OVERLAY_ALPHA)
                elif goal_flag:
                    display_with_tint = apply_overlay_tint(display_frame, GOAL_COLOR, OVERLAY_ALPHA)
                else:
                    display_with_tint = display_frame

                # Add text lines (timing) in top-left
                overlay_for_text = display_with_tint.copy()
                text_lines = [
                    "Network: {:7.1f} ms".format(net_ms),
                    "Added:   {:7.1f} ms".format(added_ms),
                    "Total:   {:7.1f} ms".format(total_ms),
                ]
                y0 = 30
                dy = 25
                x = 10
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2

                # Put a translucent rectangle behind the text for readability
                h_disp, w_disp = overlay_for_text.shape[:2]
                rect_h = dy * len(text_lines) + 10
                rect_w = 260
                rect_x1 = x - 5
                rect_y1 = 5
                rect_x2 = min(w_disp - 5, rect_x1 + rect_w)
                rect_y2 = min(h_disp - 5, rect_y1 + rect_h)
                # dark transparent rect
                bg = overlay_for_text.copy()
                cv2.rectangle(bg, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), thickness=-1)
                cv2.addWeighted(bg, 0.4, overlay_for_text, 0.6, 0, overlay_for_text)

                for i, line in enumerate(text_lines):
                    y = y0 + i * dy
                    cv2.putText(
                        overlay_for_text,
                        line,
                        (x, y),
                        font,
                        font_scale,
                        (0, 255, 0),
                        thickness,
                        cv2.LINE_AA,
                    )

                # Create right canvas matched to height with 3:2 aspect ratio
                canvas = make_right_canvas_for_height(h_disp, aspect=right_canvas_aspect)
                # Concatenate to the right
                combined = np.hstack((overlay_for_text, canvas))

                cv2.imshow("Delayed Camera Feed (with delays & overlays)", combined)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[CLIENT] 'q' pressed, exiting")
                    return

            # End inner while (frame_buffer)
        # End outer while True

    except KeyboardInterrupt:
        print("[CLIENT] KeyboardInterrupt - shutting down")
    except Exception as e:
        print(f"[CLIENT] Exception: {e}")
    finally:
        try:
            sock.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print("[CLIENT] Shutdown complete")


# -------------------------
# Small unit-test helper (runs quickly, no ROS required)
# -------------------------
def _unit_tests_quick():
    """Basic sanity tests for rotation/scaling/canvas functions using a synthetic image."""
    print("[TEST] Running quick unit tests...")
    # Create a test pattern: 200x100 (w x h) with colored quadrants
    w, h = 200, 100
    test_img = np.zeros((h, w, 3), dtype=np.uint8)
    test_img[:] = (50, 100, 150)
    cv2.putText(test_img, "TEST", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

    # Rotation
    r = rotate_frame(test_img, 45)
    assert r.shape == test_img.shape, "rotate_frame should keep same shape"

    # Scaling
    s = scale_frame(test_img, 0.5)
    assert s.shape[0] == int(round(h * 0.5)) and s.shape[1] == int(round(w * 0.5)), "scale_frame dimensions wrong"

    # Canvas
    canvas = make_right_canvas_for_height(s.shape[0], aspect=3.0/2.0)
    assert canvas.shape[0] == s.shape[0], "canvas height mismatch"
    assert canvas.shape[2] == 3, "canvas must have 3 channels"

    # Overlay tint
    tinted = apply_overlay_tint(test_img, (0, 255, 0), 0.3)
    assert tinted.shape == test_img.shape, "tint output shape mismatch"

    print("[TEST] Quick unit tests passed. Displaying test image for 1.5s...")
    display = np.hstack((test_img, tinted, canvas if canvas.shape[1]==test_img.shape[1] else scale_frame(canvas, test_img.shape[1]/canvas.shape[1])))
    cv2.imshow("unit_test_display", display)
    cv2.waitKey(1500)
    cv2.destroyAllWindows()


# -------------------------
# CLI entrypoint
# -------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Delayed camera client with ROS overlays")
    parser.add_argument("--host", default="127.0.0.1", help="camera server hostname")
    parser.add_argument("--port", type=int, default=5000, help="camera server port")
    parser.add_argument("--no-ros", action="store_true", help="run without attempting to use ROS (overrides auto-detect)")
    parser.add_argument("--rotation", type=float, default=None, help="frame rotation degrees (overrides module value)")
    parser.add_argument("--scaling", type=float, default=None, help="frame scaling factor (overrides module value)")
    parser.add_argument("--right-aspect", type=float, default=None, help="right canvas aspect ratio W/H (overrides module value)")
    parser.add_argument("--test", action="store_true", help="run quick self-tests and exit")
    args = parser.parse_args()

    if args.test:
        _unit_tests_quick()
    else:
        start_client(
            server_host=args.host,
            server_port=args.port,
            rotation_degrees=args.rotation,
            scaling_factor=args.scaling,
            right_aspect=args.right_aspect,
            use_ros=(not args.no_ros),
        )
