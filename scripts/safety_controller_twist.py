#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyControllerTwist:
    def __init__(self):
        rospy.init_node('fetch_safety_twist', anonymous=False)

        self.stop_distance = 0.6
        self.safe_to_move_forward = True

        # --- Method A: "Man-in-the-Middle" ---
        # We intercept the command, modify it, and pass it on.
        self.cmd_sub = rospy.Subscriber('/cmd_vel_teleop', Twist, self.cmd_callback)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Method A (Twist Filter) Initialized.")

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
        
        # Check center cone
        mid_index = len(ranges) // 2
        window_width = len(ranges) // 6
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        if np.min(front_ranges) < self.stop_distance:
            self.safe_to_move_forward = False
        else:
            self.safe_to_move_forward = True

    def cmd_callback(self, twist_msg):
        safe_cmd = Twist()
        
        # ACTIVE FILTERING: Modify the message object directly
        if twist_msg.linear.x > 0 and not self.safe_to_move_forward:
            safe_cmd.linear.x = 0.0 # Force 0
            safe_cmd.angular.z = twist_msg.angular.z
            rospy.logwarn_throttle(1, "Twist Filter: Stopping Command!")
        else:
            safe_cmd = twist_msg

        self.cmd_pub.publish(safe_cmd)

if __name__ == '__main__':
    try:
        SafetyControllerTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass