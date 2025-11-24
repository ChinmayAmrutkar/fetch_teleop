#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan

class SafetyControllerRunstop:
    def __init__(self):
        rospy.init_node('fetch_safety_runstop', anonymous=False)

        self.stop_distance = 0.6
        self.is_unsafe = False

        self.runstop_pub = rospy.Publisher('/enable_software_runstop', Bool, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

        rospy.loginfo("Method B (Pure Runstop) Initialized.")

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)

        # --- FIX 1: NaN/Inf to High Value ---
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0

        # --- FIX 2: FILTER GHOST NOISE (0.0 to 0.05m) ---
        ranges[ranges < 0.05] = 10.0
        
        mid_index = len(ranges) // 2
        window_width = len(ranges) // 8
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        if np.min(front_ranges) < self.stop_distance:
            if not self.is_unsafe:
                rospy.logwarn("Runstop Triggered! Disabling Motors.")
                self.is_unsafe = True
            
            self.runstop_pub.publish(True)
            
        else:
            if self.is_unsafe:
                rospy.loginfo("Path Clear. Re-enabling Motors.")
                self.is_unsafe = False
            
            self.runstop_pub.publish(False)

if __name__ == '__main__':
    try:
        SafetyControllerRunstop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass