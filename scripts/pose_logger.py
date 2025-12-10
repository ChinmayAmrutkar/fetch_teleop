#!/usr/bin/env python

import rospy
import csv
import os
import math
import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class PoseLogger:
    def __init__(self):
        rospy.init_node('pose_logger')

        # --- Parameters ---
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.log_dir = os.path.expanduser("~/pose_logs")
        
        # Calibration Offsets (Optional: to align MoCap frame to Map frame manually)
        self.offset_x = rospy.get_param('~offset_x', 0.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)

        # --- Data Holders ---
        self.latest_amcl = None
        self.latest_mocap = None

        # --- Subscribers ---
        # 1. AMCL (Estimated Position)
        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # 2. VRPN (Ground Truth)
        # Topic format is typically /vrpn_client_node/<Name>/pose
        mocap_topic = "/vrpn_client_node/{}/pose".format(self.rigid_body_name)
        self.sub_mocap = rospy.Subscriber(mocap_topic, PoseStamped, self.mocap_callback)

        # --- CSV Setup ---
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.filename = os.path.join(self.log_dir, "localization_test_{}.csv".format(timestamp))
        
        self.csv_file = open(self.filename, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["Timestamp", "AMCL_X", "AMCL_Y", "MoCap_X", "MoCap_Y", "Error_Distance"])
        
        rospy.loginfo("Logging poses to: {}".format(self.filename))
        rospy.loginfo("Listening to MoCap topic: {}".format(mocap_topic))

    def amcl_callback(self, msg):
        self.latest_amcl = msg
        self.log_data()

    def mocap_callback(self, msg):
        self.latest_mocap = msg
        # We don't log here because MoCap is 100Hz+ (too fast). 
        # We log only when AMCL updates (the data we are testing).

    def log_data(self):
        # Only log if we have data from both sources
        if self.latest_amcl and self.latest_mocap:
            t = rospy.Time.now().to_sec()
            
            # AMCL Data
            ax = self.latest_amcl.pose.pose.position.x
            ay = self.latest_amcl.pose.pose.position.y
            
            # MoCap Data (Applied Offsets)
            mx = self.latest_mocap.pose.position.x + self.offset_x
            my = self.latest_mocap.pose.position.y + self.offset_y
            
            # Simple Euclidean Error
            # NOTE: This assumes Map Frame and Optitrack Frame are aligned!
            error = math.sqrt((ax - mx)**2 + (ay - my)**2)
            
            self.writer.writerow([t, ax, ay, mx, my, error])
            
            # Print status to terminal
            rospy.loginfo_throttle(1, "AMCL: ({:.2f}, {:.2f}) | MoCap: ({:.2f}, {:.2f}) | Err: {:.3f}m".format(ax, ay, mx, my, error))

    def shutdown(self):
        self.csv_file.close()
        rospy.loginfo("Log file closed.")

if __name__ == '__main__':
    node = PoseLogger()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()