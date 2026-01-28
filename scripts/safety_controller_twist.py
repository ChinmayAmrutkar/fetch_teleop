#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool # Added for status publishing
from sensor_msgs.msg import LaserScan

class SafetyControllerTwist:
    def __init__(self):
        rospy.init_node('fetch_safety_twist', anonymous=False)

        self.stop_distance = 0.3
        self.safe_to_move_forward = True

        # Listen to the (potentially delayed) command stream
        self.cmd_sub = rospy.Subscriber('/cmd_vel_teleop', Twist, self.cmd_callback)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # --- NEW: Publish Safety Status ---
        # True = Safety Triggered (Stop), False = Safe
        self.status_pub = rospy.Publisher('/safety_status', Bool, queue_size=1)

        rospy.loginfo("Method A (Real-Time Safety) Initialized.")

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        
        # Filter Logic
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        ranges[ranges < 0.05] = 10.0
        
        mid_index = len(ranges) // 2
        window_width = len(ranges) // 6
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        # CONTINUOUS SAFETY CHECK
        if np.min(front_ranges) < self.stop_distance:
            self.safe_to_move_forward = False
        else:
            self.safe_to_move_forward = True

        # Publish Status (True if UNSAFE/STOPPED)
        self.status_pub.publish(not self.safe_to_move_forward)

    def cmd_callback(self, twist_msg):
        safe_cmd = Twist()
        
        if twist_msg.linear.x > 0 and not self.safe_to_move_forward:
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = twist_msg.angular.z
            rospy.logwarn_throttle(1, "Safety Controller: Blocking Command (Obstacle Ahead)")
        else:
            safe_cmd = twist_msg

        self.cmd_pub.publish(safe_cmd)

if __name__ == '__main__':
    try:
        SafetyControllerTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass