#!/usr/bin/env python

import rospy
import csv
import os
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class MocapComparer:
    def __init__(self):
        rospy.init_node('pose_logger', anonymous=True)

        # --- CONFIGURATION ---
        # Allow these to be set via Launch file
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        
        # Safe Directory Setup
        self.log_dir = os.path.expanduser("~/pose_logs")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        self.log_file_path = os.path.join(self.log_dir, 'amcl_vs_mocap_log.csv')
        self.amcl_path_log = os.path.join(self.log_dir, 'amcl_path.csv')
        self.mocap_path_log = os.path.join(self.log_dir, 'mocap_path.csv')

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_link' 

        # --- TF LISTENER ---
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        self.latest_mocap_pose = None
        self.initial_amcl_pose = None
        self.initial_mocap_pose = None

        # --- SUBSCRIBER ---
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        rospy.loginfo("Subscribing to Mocap on: %s", self.mocap_topic)

        # --- PUBLISHERS (VIZ) ---
        self.amcl_path_pub = rospy.Publisher('/viz_path_amcl', Path, queue_size=10)
        self.mocap_path_pub = rospy.Publisher('/viz_path_mocap', Path, queue_size=10)
        self.amcl_path_msg = Path()
        self.mocap_path_msg = Path()
        self.amcl_path_msg.header.frame_id = self.map_frame
        self.mocap_path_msg.header.frame_id = self.map_frame

        # --- CSV SETUP ---
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'mocap_x', 'mocap_y', 'error_dist'])
        rospy.loginfo("Logging to: %s", self.log_file_path)

        # --- LOOP ---
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def mocap_callback(self, msg):
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        # 1. Get AMCL Position from TF
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            amcl_x, amcl_y = trans[0], trans[1]
            _, _, amcl_yaw = euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        if self.latest_mocap_pose is None:
            return

        # 2. Capture Initial AMCL Pose
        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = [amcl_x, amcl_y, amcl_yaw]
            return

        # 3. Get Current Mocap
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x, self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z, self.latest_mocap_pose.orientation.w])
        
        # 4. Extract Initial Mocap Yaw
        _, _, init_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x, self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z, self.initial_mocap_pose.orientation.w])

        # 5. NORMALIZE (Shift to 0,0)
        # Shift Mocap
        norm_mocap_x = mocap_x - self.initial_mocap_pose.position.x
        norm_mocap_y = mocap_y - self.initial_mocap_pose.position.y
        
        # Shift AMCL
        norm_amcl_x = amcl_x - self.initial_amcl_pose[0]
        norm_amcl_y = amcl_y - self.initial_amcl_pose[1]

        # 6. ROTATE AMCL to align with Mocap Heading
        # Calculate offset between AMCL start heading and Mocap start heading
        dyaw = init_mocap_yaw - self.initial_amcl_pose[2]
        
        # Apply rotation to AMCL vector
        aligned_amcl_x = norm_amcl_x * math.cos(dyaw) - norm_amcl_y * math.sin(dyaw)
        aligned_amcl_y = norm_amcl_x * math.sin(dyaw) + norm_amcl_y * math.cos(dyaw)

        # 7. Error Calculation
        error_dist = math.sqrt((aligned_amcl_x - norm_mocap_x)**2 + (aligned_amcl_y - norm_mocap_y)**2)

        # 8. Log
        self.csv_writer.writerow([rospy.Time.now().to_sec(), aligned_amcl_x, aligned_amcl_y, norm_mocap_x, norm_mocap_y, error_dist])
        rospy.loginfo_throttle(1, "Err: {:.3f}m | AMCL: {:.2f} {:.2f} | MoCap: {:.2f} {:.2f}".format(error_dist, aligned_amcl_x, aligned_amcl_y, norm_mocap_x, norm_mocap_y))

        # 9. Visualize Path (Aligned)
        self.publish_path(self.amcl_path_pub, self.amcl_path_msg, aligned_amcl_x, aligned_amcl_y)
        self.publish_path(self.mocap_path_pub, self.mocap_path_msg, norm_mocap_x, norm_mocap_y)

    def publish_path(self, pub, path_msg, x, y):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    MocapComparer().run()