#!/usr/bin/env python

import rospy
from collections import deque
from geometry_msgs.msg import Twist

class CmdDelay:
    def __init__(self):
        rospy.init_node('cmd_delay_node')

        # --- UPDATED: Get Delay from Parameter Server ---
        # This ensures it matches the camera delay set in the launch file
        self.delay = rospy.get_param('~delay', 1.0)
        
        self.buffer = deque()

        # Listen to the RAW input from keyboard
        self.cmd_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_callback)
        
        # Publish to the topic the Admittance Controller listens to
        self.cmd_pub = rospy.Publisher('/cmd_vel_delayed', Twist, queue_size=10)

        # Timer to process buffer at 50Hz
        self.timer = rospy.Timer(rospy.Duration(0.02), self.process_buffer)

        rospy.loginfo("CMD Delay Node Started. Latency: {} seconds".format(self.delay))

    def cmd_callback(self, msg):
        now = rospy.Time.now()
        self.buffer.append((now, msg))

    def process_buffer(self, event):
        if not self.buffer:
            return

        now = rospy.Time.now()
        
        while self.buffer:
            timestamp, msg = self.buffer[0]
            age = (now - timestamp).to_sec()

            if age >= self.delay:
                self.buffer.popleft() # Remove from queue
                self.cmd_pub.publish(msg) 
            else:
                break

if __name__ == '__main__':
    CmdDelay()
    rospy.spin()