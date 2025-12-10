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