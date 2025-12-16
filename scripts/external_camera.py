#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import struct
import time
from collections import deque
import cv2
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Set the desired total delay (seconds) here:
total_delay = 0.001  # e.g., 500 ms total latency

def recv_exact(sock, count):
    """Receive exactly `count` bytes from the socket."""
    buf = b""
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk:
            raise ValueError("Socket closed while receiving data")
        buf += chunk
    return buf

def start_client(server_host="192.168.0.172", server_port=5000):
    global total_delay

    # --- ROS SETUP ---
    rospy.init_node('external_camera_node', anonymous=True)
    image_pub = rospy.Publisher('/external_camera/image_delayed', Image, queue_size=10)
    bridge = CvBridge()
    rospy.loginfo("External Camera Node Started. Publishing to /external_camera/image_delayed")

    # Socket Setup
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rospy.loginfo("[CLIENT] Connecting to {}:{} ...".format(server_host, server_port))
    
    try:
        sock.connect((server_host, server_port))
        rospy.loginfo("[CLIENT] Connected")
    except socket.error as e:
        rospy.logerr("Could not connect to camera server: {}".format(e))
        return

    frame_buffer = deque()
    clock_offset = None

    try:
        while not rospy.is_shutdown():
            # --- Receive Header ---
            try:
                header = recv_exact(sock, 8 + 4)
            except ValueError:
                break
                
            server_ts, frame_size = struct.unpack("!dI", header)

            # --- Receive JPEG ---
            jpg_bytes = recv_exact(sock, frame_size)
            arrival_time_client = time.time()

            # --- Decode ---
            nparr = np.frombuffer(jpg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # --- Clock Sync ---
            if clock_offset is None:
                clock_offset = arrival_time_client - server_ts
                print("[CLIENT] Estimated clock offset: {:.4f} s".format(clock_offset))

            ts_client = server_ts + clock_offset
            target_display_time = ts_client + total_delay

            # Buffer the frame
            frame_buffer.append((server_ts, arrival_time_client, target_display_time, frame))

            # --- Process Buffer ---
            while frame_buffer:
                server_ts0, arrival0, t_display0, fr = frame_buffer[0]
                now = time.time()
                wait = t_display0 - now

                if wait > 0:
                    # Wait if needed (small sleep to prevent CPU burn)
                    time.sleep(min(wait, 0.005))
                    break

                # Frame is ready to show/publish
                frame_buffer.popleft()

                # Calculate stats for overlay
                ts_client0 = server_ts0 + clock_offset
                network_delay = max(arrival0 - ts_client0, 0.0)
                total_delay_measured = max(now - ts_client0, 0.0)
                added_delay = max(total_delay_measured - network_delay, 0.0)

                net_ms = network_delay * 1000.0
                added_ms = added_delay * 1000.0
                total_ms = total_delay_measured * 1000.0

                # --- Create Overlay ---
                overlay = fr.copy()
                text_lines = [
                    "Network: {:7.1f}   ms".format(net_ms),
                    "Added:   {:7.1f}   ms".format(added_ms),
                    "Total:   {:7.1f}   ms".format(total_ms),
                ]

                y0, dy, x = 30, 25, 10
                for i, line in enumerate(text_lines):
                    cv2.putText(overlay, line, (x, y0 + i * dy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                # --- 1. PUBLISH TO ROS ---
                try:
                    # Convert OpenCV (BGR) to ROS Message
                    ros_msg = bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                    ros_msg.header.stamp = rospy.Time.now()
                    ros_msg.header.frame_id = "external_camera_link"
                    image_pub.publish(ros_msg)
                except Exception as e:
                    rospy.logerr("CvBridge Error: {}".format(e))

                # --- 2. DISPLAY (Optional) ---
                # We wrap this in try-except because it fails if run via SSH without X11
                try:
                    cv2.imshow("Delayed Camera Feed", overlay)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        rospy.signal_shutdown("User pressed Q")
                        return
                except Exception:
                    pass

    except Exception as e:
        rospy.logerr("Error in loop: {}".format(e))
    finally:
        sock.close()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        rospy.loginfo("[CLIENT] Shutdown complete")

if __name__ == "__main__":
    # Ensure this IP matches your Camera Server
    start_client(server_host="192.168.0.172", server_port=5000)