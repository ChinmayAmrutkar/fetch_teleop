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

        # --- Method B: "PURE Asynchronous Trigger" ---
        # We ONLY publish the runstop flag. 
        # We do NOT inject Twist=0. We trust the robot driver to handle the stop.
        self.runstop_pub = rospy.Publisher('/enable_software_runstop', Bool, queue_size=10)
        
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)

        rospy.loginfo("Method B (Pure Runstop) Initialized.")

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
        
        mid_index = len(ranges) // 2
        window_width = len(ranges) // 6
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        # --- Logic: Trigger the Flag ---
        if np.min(front_ranges) < self.stop_distance:
            if not self.is_unsafe:
                rospy.logwarn("Runstop Triggered! Disabling Motors.")
                self.is_unsafe = True
            
            # Publish Runstop TRUE (Hardware Stop)
            self.runstop_pub.publish(True)
            
        else:
            if self.is_unsafe:
                rospy.loginfo("Path Clear. Re-enabling Motors.")
                self.is_unsafe = False
            
            # Publish Runstop FALSE (Enable Motors)
            self.runstop_pub.publish(False)

if __name__ == '__main__':
    try:
        SafetyControllerRunstop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass