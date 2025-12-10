# #!/usr/bin/env python

# import rospy
# from collections import deque
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Image

# class LatencySimulator:
#     def __init__(self):
#         rospy.init_node('latency_simulator_node')

#         # Parameter: Delay in seconds
#         self.delay = 1.0
        
#         # Buffers
#         self.cmd_buffer = deque()
#         self.img_buffer = deque()

#         # --- 1. VELOCITY (CMD_VEL) SETUP ---
#         # Listen to RAW input from keyboard
#         self.cmd_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_callback)
#         # Publish to Teleop topic (which the Safety Controller listens to)
#         self.cmd_pub = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=10)

#         # --- 2. CAMERA (IMAGE) SETUP ---
#         # Listen to Real Camera
#         self.img_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.img_callback)
#         # Publish to Delayed Topic (for RViz)
#         self.img_pub = rospy.Publisher('/head_camera/rgb/image_raw_delayed', Image, queue_size=10)

#         # Timer to process buffers at 50Hz
#         self.timer = rospy.Timer(rospy.Duration(0.02), self.process_buffers)

#         rospy.loginfo("Latency Simulator Started. Delay: {}s. Buffering cmd_vel and images.".format(self.delay))

#     def cmd_callback(self, msg):
#         now = rospy.get_rostime()
#         self.cmd_buffer.append((now, msg))

#     def img_callback(self, msg):
#         now = rospy.get_rostime()
#         self.img_buffer.append((now, msg))

#     def process_buffers(self, event):
#         now = rospy.get_rostime()
        
#         # --- PROCESS CMD BUFFER ---
#         while self.cmd_buffer:
#             timestamp, msg = self.cmd_buffer[0]
#             age = (now - timestamp).to_sec()
            
#             if age >= self.delay:
#                 self.cmd_buffer.popleft()
#                 self.cmd_pub.publish(msg)
#             else:
#                 break
        
#         # --- PROCESS IMAGE BUFFER ---
#         while self.img_buffer:
#             timestamp, msg = self.img_buffer[0]
#             age = (now - timestamp).to_sec()
            
#             if age >= self.delay:
#                 self.img_buffer.popleft()
#                 self.img_pub.publish(msg)
#             else:
#                 break

# if __name__ == '__main__':
#     LatencySimulator()
#     rospy.spin()

#!/usr/bin/env python

import rospy
from collections import deque
from geometry_msgs.msg import Twist

class CmdDelay:
    def __init__(self):
        rospy.init_node('cmd_delay_node')

        # Parameter: Delay in seconds
        self.delay = 1.0
        self.buffer = deque()

        # Listen to the RAW input from keyboard
        self.cmd_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_callback)
        
        # Publish to the topic the Safety Controller listens to
        self.cmd_pub = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=10)

        # Timer to process buffer at 50Hz
        self.timer = rospy.Timer(rospy.Duration(0.02), self.process_buffer)

        rospy.loginfo("Control Delay Node Started. Latency: {} seconds".format(self.delay))

    def cmd_callback(self, msg):
        # Timestamp the incoming command and store it
        now = rospy.get_rostime()
        self.buffer.append((now, msg))

    def process_buffer(self, event):
        if not self.buffer:
            return

        now = rospy.get_rostime()
        
        # Process all ready messages
        while self.buffer:
            timestamp, msg = self.buffer[0]
            age = (now - timestamp).to_sec()

            if age >= self.delay:
                self.buffer.popleft() # Remove from queue
                self.cmd_pub.publish(msg) # Send to Safety Node
            else:
                # If oldest message isn't ready, nothing is
                break

if __name__ == '__main__':
    CmdDelay()
    rospy.spin()