#!/usr/bin/env python

import rospy
from collections import deque
from geometry_msgs.msg import Twist

class CmdDelay:
    def __init__(self):
        rospy.init_node('cmd_delay_node')

        # Parameter: Delay in seconds
        self.delay = 0.5 # 500 milliseconds
        self.buffer = deque()

        # Listen to the RAW input from keyboard
        self.cmd_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_callback)
        
        # Publish to the topic the Admittance Controller listens to
        self.cmd_pub = rospy.Publisher('/cmd_vel_delayed', Twist, queue_size=10)

        # Timer to process buffer at 50Hz
        self.timer = rospy.Timer(rospy.Duration(0.02), self.process_buffer)

        rospy.loginfo("DEBUG MODE: Delay Node Started. Waiting for /cmd_vel_raw...")

    def cmd_callback(self, msg):
        # DEBUG: Confirm we got input
        rospy.loginfo_throttle(1, "Input Received! Buffering...")
        
        # Use Time.now() which is safer than get_rostime() on some setups
        now = rospy.Time.now()
        self.buffer.append((now, msg))

    def process_buffer(self, event):
        if not self.buffer:
            return

        now = rospy.Time.now()
        
        # Process all ready messages
        while self.buffer:
            timestamp, msg = self.buffer[0]
            age = (now - timestamp).to_sec()

            if age >= self.delay:
                # DEBUG: Confirm we are releasing
                rospy.loginfo_throttle(1, "Releasing message! Delay satisfied.")
                
                self.buffer.popleft() # Remove from queue
                self.cmd_pub.publish(msg) 
            else:
                # If oldest message isn't ready, nothing is
                break

if __name__ == '__main__':
    CmdDelay()
    rospy.spin()