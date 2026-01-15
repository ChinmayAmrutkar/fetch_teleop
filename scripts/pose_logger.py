#!/usr/bin/env python

import rospy
import csv
import os
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class MocapComparer:
    def __init__(self):
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        
        # --- PATHS ---
        self.log_dir = os.path.expanduser("/home/chinmay/fetch_teleop_ws/pose_logs")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self.log_file_path = os.path.join(self.log_dir, 'amcl_vs_mocap_log.csv')

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        # --- TF LISTENER ---
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        self.latest_mocap_pose = None
        self.initial_amcl_pose = None
        self.initial_mocap_pose = None
        self.current_input = Twist() # Store latest user input

        # --- SUBSCRIBERS ---
        # 1. MoCap
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        # 2. Input (User Joystick/Keyboard)
        rospy.Subscriber('/cmd_vel_raw', Twist, self.input_callback)
        
        rospy.loginfo("Subscribing to Mocap: %s", self.mocap_topic)
        rospy.loginfo("Logging to: %s", self.log_dir)

        # --- PUBLISHERS (Viz) ---
        self.amcl_path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10)
        self.mocap_path_pub = rospy.Publisher('/mocap_path', Path, queue_size=10)
        
        self.amcl_path = Path()
        self.mocap_path = Path()
        self.amcl_path.header.frame_id = self.map_frame
        self.mocap_path.header.frame_id = self.map_frame

        # --- CSV SETUP ---
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        # Added Input_JX, Input_JY to header
        self.csv_writer.writerow([
            'timestamp', 
            'Input_JX', 'Input_JY',
            'amcl_x', 'amcl_y', 'amcl_yaw', 
            'mocap_x', 'mocap_y', 'mocap_yaw', 
            'error_x', 'error_y', 'error_yaw', 'error_distance'
        ])
        
        # --- LOOP ---
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def input_callback(self, msg):
        self.current_input = msg

    def mocap_callback(self, msg):
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        # 1. Get AMCL Pose
        try:
            (amcl_trans, amcl_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            amcl_x, amcl_y = amcl_trans[0], amcl_trans[1]
            _, _, amcl_yaw = euler_from_quaternion(amcl_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        if self.latest_mocap_pose is None:
            return

        # 2. Initialization
        # Extract initial mocap yaw
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x, self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z, self.initial_mocap_pose.orientation.w
        ])

        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = [amcl_x, amcl_y, amcl_yaw]
            rospy.loginfo("Captured Initial AMCL Pose. Starting Sync.")
            return

        # 3. Process MoCap (KEEP RAW)
        # We do NOT normalize this to 0,0. We keep it in the original frame.
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x, self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z, self.latest_mocap_pose.orientation.w
        ])

        # 4. Process AMCL (ALIGN TO MOCAP FRAME)
        # Step A: Normalize AMCL to (0,0) relative to its start
        norm_amcl_x = amcl_x - self.initial_amcl_pose[0]
        norm_amcl_y = amcl_y - self.initial_amcl_pose[1]
        
        # Step B: Rotate AMCL to match MoCap's starting orientation
        dyaw = initial_mocap_yaw - self.initial_amcl_pose[2]
        rot_amcl_x, rot_amcl_y = self.rotate_2d_vector(norm_amcl_x, norm_amcl_y, dyaw)
        
        # Step C: SHIFT to MoCap's starting position (The "Overlay" Step)
        final_amcl_x = rot_amcl_x + self.initial_mocap_pose.position.x
        final_amcl_y = rot_amcl_y + self.initial_mocap_pose.position.y
        final_amcl_yaw = amcl_yaw + dyaw # Approximation of aligned yaw

        # 5. Calculate Error (Difference between overlays)
        error_x = final_amcl_x - mocap_x
        error_y = final_amcl_y - mocap_y
        error_dist = math.sqrt(error_x**2 + error_y**2)
        error_yaw = math.atan2(math.sin(final_amcl_yaw - mocap_yaw), math.cos(final_amcl_yaw - mocap_yaw))

        # 6. Log
        rospy.loginfo_throttle(1, "Err: %.3f | AMCL(Aligned): %.2f %.2f | MOCAP(Raw): %.2f %.2f", error_dist, final_amcl_x, final_amcl_y, mocap_x, mocap_y)

        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            self.current_input.linear.x, self.current_input.angular.z,
            final_amcl_x, final_amcl_y, final_amcl_yaw,
            mocap_x, mocap_y, mocap_yaw,
            error_x, error_y, error_yaw, error_dist
        ])

        # 7. Visualize
        current_time = rospy.Time.now()
        # Publish the ALIGNED AMCL path (Green line will appear over Red line)
        self.update_path(self.amcl_path, final_amcl_x, final_amcl_y, final_amcl_yaw, current_time)
        self.amcl_path_pub.publish(self.amcl_path)

        # Publish the RAW MoCap path
        self.update_path(self.mocap_path, mocap_x, mocap_y, mocap_yaw, current_time)
        self.mocap_path_pub.publish(self.mocap_path)

    def update_path(self, path_msg, x, y, yaw, timestamp):
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 
        path_msg.poses.append(pose)
        path_msg.header.stamp = timestamp

    def rotate_2d_vector(self, x, y, angle_rad):
        new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
        return new_x, new_y

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node and saving paths...")
        self.csv_file.close()

    def run(self):
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()

if __name__ == '__main__':
    comparer = MocapComparer()
    comparer.run()