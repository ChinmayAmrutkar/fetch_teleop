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

        self.cmd_sub = rospy.Subscriber('/cmd_vel_teleop', Twist, self.cmd_callback)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Method A (Twist Filter) Initialized.")

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        
        # --- FIX 1: NaN/Inf to High Value ---
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        
        # --- FIX 2: FILTER GHOST NOISE (0.0 to 0.05m) ---
        # Any value less than 5cm is likely a driver error or self-reflection.
        # We treat it as "clear" (10.0m)
        ranges[ranges < 0.05] = 10.0
        
        # Check center cone
        mid_index = len(ranges) // 2
        window_width = len(ranges) // 8
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        # Log the minimum value for debugging
        min_val = np.min(front_ranges)
        # rospy.loginfo_throttle(0.5, "Min Dist: {:.3f}".format(min_val))
        
        if min_val < self.stop_distance:
            self.safe_to_move_forward = False
        else:
            self.safe_to_move_forward = True

    def cmd_callback(self, twist_msg):
        safe_cmd = Twist()
        
        if twist_msg.linear.x > 0 and not self.safe_to_move_forward:
            safe_cmd.linear.x = 0.0
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