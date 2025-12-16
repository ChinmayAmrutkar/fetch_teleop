#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import struct
import time
from collections import deque

import cv2
import numpy as np

# Set the desired total delay (seconds) here:
total_delay = 0.001  # e.g., 500 ms total latency

# If you already know the "base" end-to-end latency (capture+encode+network+decode),
# you can use it just as a sanity check. The script will still use timestamps to
# approximate total_delay.
estimated_base_latency = 0.100  # seconds (for your reference / tuning)


def recv_exact(sock, count):
    """Receive exactly `count` bytes from the socket."""
    buf = b""
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk:
            raise ValueError("Socket closed while receiving data")
        buf += chunk
    return buf


def start_client(server_host="127.0.0.1", server_port=5000):
    """
    Connect to the camera server, receive timestamped frames, and display them
    so that the total delay (capture -> display) is approximately `total_delay`.

    The script also overlays:
      - Network delay (capture -> arrival)
      - Added delay (arrival -> display)
      - Total delay (capture -> display)
    on each displayed frame.
    """
    global total_delay

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("[CLIENT] Connecting to {}:{} ...".format(server_host,server_port))
    sock.connect((server_host, server_port))
    print("[CLIENT] Connected")

    # Buffer: (server_ts, arrival_time_client, target_display_time, frame)
    frame_buffer = deque()
    # Estimate clock offset: client_time ≈ server_time + offset
    clock_offset = None

    try:
        while True:
            # --- Receive one frame from server ---

            # Header: [8-byte double: server_ts][4-byte uint: frame_size]
            header = recv_exact(sock, 8 + 4)
            server_ts, frame_size = struct.unpack("!dI", header)

            # JPEG bytes
            jpg_bytes = recv_exact(sock, frame_size)

            # Arrival time on client clock (when we have full frame data)
            arrival_time_client = time.time()

            # Decode JPEG
            nparr = np.frombuffer(jpg_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                print("[CLIENT] Failed to decode frame")
                continue

            # --- Clock sync (first frame) ---
            if clock_offset is None:
                # client_time ≈ server_ts + offset → offset ≈ arrival_time_client - server_ts
                clock_offset = arrival_time_client - server_ts
                print("[CLIENT] Estimated clock offset: {clock_offset:.4f} s")

            # Server timestamp expressed in client time base
            ts_client = server_ts + clock_offset

            # When should we show this frame so total delay = total_delay?
            # Target display time in client clock:
            target_display_time = ts_client + total_delay

            # Store frame + timing info
            frame_buffer.append((server_ts, arrival_time_client, target_display_time, frame))

            # --- Display any frames that are now due ---
            while frame_buffer:
                server_ts0, arrival0, t_display0, fr = frame_buffer[0]
                now = time.time()
                wait = t_display0 - now

                if wait > 0:
                    # Not yet time to display this frame; wait a bit
                    time.sleep(min(wait, 0.005))
                    break

                # Time to display (or we are late), pop from buffer
                frame_buffer.popleft()

                # Compute delays for overlay, using the same ts_client base
                ts_client0 = server_ts0 + clock_offset

                network_delay = max(arrival0 - ts_client0, 0.0)       # capture -> arrival
                total_delay_measured = max(now - ts_client0, 0.0)      # capture -> display
                added_delay = max(total_delay_measured - network_delay, 0.0)  # arrival -> display

                # Format delays in ms for readability
                net_ms = network_delay * 1000.0
                added_ms = added_delay * 1000.0
                total_ms = total_delay_measured * 1000.0

                # --- Overlay text on frame ---
                overlay = fr.copy()
                text_lines = [
                    "Network: {:7.1f}   ms".format(net_ms),
                    "Added:   {:7.1f}   ms".format(added_ms),
                    "Total:   {:7.1f}   ms".format(total_ms),
                ]

                y0 = 30
                dy = 25
                x = 10
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2

                # Optional: draw background rectangle for readability
                # Compute rectangle height
                # rect_height = dy * len(text_lines) + 10
                # cv2.rectangle(
                #     overlay,
                #     (5, 5),
                #     (300, 5 + rect_height),
                #     (0, 0, 0),
                #     thickness=-1,
                # )

                # Draw each line
                for i, line in enumerate(text_lines):
                    y = y0 + i * dy
                    cv2.putText(
                        overlay,
                        line,
                        (x, y),
                        font,
                        font_scale,
                        (0, 255, 0),
                        thickness,
                        cv2.LINE_AA,
                    )

                cv2.imshow("Delayed Camera Feed (with delays)", overlay)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return

    #except (ConnectionError, KeyboardInterrupt):
    #    print("[CLIENT] Stopping...")
    except Exception as e:
        print(e)
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print("[CLIENT] Shutdown complete")


# def start_client(server_host="SERVER_IP_HERE", server_port=5000):
#     """
#     Connect to the camera server, receive timestamped frames, and display them
#     with an additional buffer so that the *total* delay ~= total_delay.

#     Assumes server and client clocks are roughly synchronized (e.g., via NTP).
#     """
#     global total_delay

#     sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     print(f"[CLIENT] Connecting to {server_host}:{server_port} ...")
#     sock.connect((server_host, server_port))
#     print("[CLIENT] Connected")

#     # Buffer to hold incoming frames until it is time to display them
#     # Each element: (server_timestamp, frame)
#     frame_buffer = deque()

#     # Estimate clock offset: client_time ≈ server_time + offset
#     clock_offset = None

#     try:
#         while True:
#             # Read header
#             header = recv_exact(sock, 8 + 4)
#             server_ts, frame_size = struct.unpack("!dI", header)

#             # Read JPEG frame
#             jpg_bytes = recv_exact(sock, frame_size)
#             nparr = np.frombuffer(jpg_bytes, np.uint8)
#             frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
#             if frame is None:
#                 print("[CLIENT] Failed to decode frame")
#                 continue

#             now_client = time.time()

#             if clock_offset is None:
#                 # On first frame, estimate offset between clocks
#                 # client_time ≈ server_ts + offset -> offset ≈ now_client - server_ts
#                 clock_offset = now_client - server_ts
#                 print(f"[CLIENT] Estimated clock offset: {clock_offset:.4f} s")

#             # Convert server timestamp to local time base
#             frame_server_time_on_client_clock = server_ts + clock_offset

#             # When should we display this frame?
#             target_display_time = frame_server_time_on_client_clock + total_delay

#             # Add to buffer
#             frame_buffer.append((target_display_time, frame))

#             # Display all frames that are now "due"
#             while frame_buffer:
#                 t_display, fr = frame_buffer[0]
#                 now = time.time()
#                 wait = t_display - now

#                 if wait > 0:
#                     # Not ready to display yet; sleep a bit and break to receive more data
#                     # For smoother timing, sleep only a short interval
#                     time.sleep(min(wait, 0.005))
#                     break
#                 else:
#                     # Time has passed or we are behind schedule; show it now
#                     frame_buffer.popleft()
#                     cv2.imshow("Delayed Camera Feed", fr)
#                     # 1 ms wait so OpenCV can process window events
#                     if cv2.waitKey(1) & 0xFF == ord("q"):
#                         return

#     except (ConnectionError, KeyboardInterrupt):
#         print("[CLIENT] Stopping...")
#     finally:
#         sock.close()
#         cv2.destroyAllWindows()
#         print("[CLIENT] Shutdown complete")


if __name__ == "__main__":
    # Replace with the actual IP of the server machine on your LAN
    #start_client(server_host="192.168.1.10", server_port=5000)
    # start_client(server_host="127.0.0.1", server_port=5000)
    start_client(server_host="192.168.0.172", server_port=5000)

