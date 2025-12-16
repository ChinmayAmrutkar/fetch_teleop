#!/usr/bin/env python

# ROS and utility imports
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
        rospy.init_node('mocap_comparer', anonymous=True)

        # --- CONFIGURATION ---
        # Changed from hardcoded 'Fetch1' to a parameter (Default: Fetch)
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        
        self.log_dir = os.path.expanduser("~/pose_logs")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self.log_file_path = os.path.join(self.log_dir, 'amcl_vs_mocap_log.csv')
        self.amcl_path_log_file = os.path.join(self.log_dir, 'amcl_path_log.csv')
        self.mocap_path_log_file = os.path.join(self.log_dir, 'mocap_path_log.csv')

        # TF frames used for transform lookup
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        # --- TF LISTENER ---
        self.tf_listener = tf.TransformListener()

        # --- DATA STORAGE ---
        self.latest_mocap_pose = None
        self.initial_amcl_pose = None
        self.initial_mocap_pose = None

        # --- SUBSCRIBER for Mocap Ground Truth ---
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        rospy.loginfo("Subscribing to Mocap on: %s", self.mocap_topic)
        rospy.loginfo("Logging to: %s", self.log_dir)

        # --- PUBLISHERS for Visualization ---
        # Using the topic names you requested
        self.amcl_path_pub = rospy.Publisher('/amcl_path', Path, queue_size=10)
        self.mocap_path_pub = rospy.Publisher('/mocap_path', Path, queue_size=10)
        
        self.amcl_path = Path()
        self.mocap_path = Path()
        self.amcl_path.header.frame_id = self.map_frame
        self.mocap_path.header.frame_id = self.map_frame

        # --- CSV SETUP ---
        self.csv_file = open(self.log_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'amcl_x', 'amcl_y', 'amcl_yaw', 'mocap_x', 'mocap_y', 'mocap_yaw', 'error_x', 'error_y', 'error_yaw', 'error_distance'])
        
        # --- MAIN PROCESSING LOOP ---
        rospy.Timer(rospy.Duration(0.5), self.processing_loop)

    def mocap_callback(self, msg):
        """Callback to update Mocap pose."""
        self.latest_mocap_pose = msg.pose
        if self.initial_mocap_pose is None:
            self.initial_mocap_pose = self.latest_mocap_pose
            rospy.loginfo("Captured initial Mocap pose.")

    def processing_loop(self, event):
        """Main loop: fetch AMCL pose, compare with Mocap, log & visualize."""
        
        # 1. Get AMCL Pose from TF
        try:
            (amcl_trans, amcl_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            amcl_x, amcl_y, amcl_z = amcl_trans
            amcl_roll, amcl_pitch, amcl_yaw = euler_from_quaternion(amcl_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # We fail silently/quietly here as requested (no robust init warnings)
            return

        # If no mocap pose received yet, wait
        if self.latest_mocap_pose is None:
            return

        # 2. Initialization: Capture Start Poses
        # Extract initial mocap yaw
        _, _, initial_mocap_yaw = euler_from_quaternion([
            self.initial_mocap_pose.orientation.x, self.initial_mocap_pose.orientation.y,
            self.initial_mocap_pose.orientation.z, self.initial_mocap_pose.orientation.w
        ])

        if self.initial_amcl_pose is None:
            self.initial_amcl_pose = [amcl_x, amcl_y, amcl_yaw]
            rospy.loginfo("Captured Initial AMCL Pose.")
            return

        # 3. Normalization & Alignment
        
        # Current Mocap Pose
        mocap_x = self.latest_mocap_pose.position.x
        mocap_y = self.latest_mocap_pose.position.y
        _, _, mocap_yaw = euler_from_quaternion([
            self.latest_mocap_pose.orientation.x, self.latest_mocap_pose.orientation.y,
            self.latest_mocap_pose.orientation.z, self.latest_mocap_pose.orientation.w
        ])

        # Normalize Mocap (Shift to 0,0)
        norm_mocap_x = mocap_x - self.initial_mocap_pose.position.x
        norm_mocap_y = mocap_y - self.initial_mocap_pose.position.y
        norm_mocap_yaw = mocap_yaw - initial_mocap_yaw

        # Normalize AMCL (Shift to 0,0)
        norm_amcl_x = amcl_x - self.initial_amcl_pose[0]
        norm_amcl_y = amcl_y - self.initial_amcl_pose[1]
        norm_amcl_yaw = amcl_yaw - self.initial_amcl_pose[2]

        # Rotate AMCL to match Mocap Heading
        dyaw = initial_mocap_yaw - self.initial_amcl_pose[2]
        aligned_amcl_x, aligned_amcl_y = rotate_2d_vector(norm_amcl_x, norm_amcl_y, dyaw)

        # 4. Error Metrics
        error_x = aligned_amcl_x - norm_mocap_x
        error_y = aligned_amcl_y - norm_mocap_y
        error_dist = math.sqrt(error_x**2 + error_y**2)
        error_yaw = math.atan2(math.sin(norm_amcl_yaw - norm_mocap_yaw), math.cos(norm_amcl_yaw - norm_mocap_yaw))

        # 5. Logging
        rospy.loginfo_throttle(1, "Err: %.3f | AMCL: %.2f %.2f | MOCAP: %.2f %.2f", error_dist, aligned_amcl_x, aligned_amcl_y, norm_mocap_x, norm_mocap_y)

        self.csv_writer.writerow([
            rospy.Time.now().to_sec(),
            aligned_amcl_x, aligned_amcl_y, norm_amcl_yaw,
            norm_mocap_x, norm_mocap_y, norm_mocap_yaw,
            error_x, error_y, error_yaw, error_dist
        ])

        # 6. Visualization (Publish Paths)
        current_time = rospy.Time.now()

        # AMCL Path
        self.update_path(self.amcl_path, aligned_amcl_x, aligned_amcl_y, amcl_yaw, current_time)
        self.amcl_path_pub.publish(self.amcl_path)

        # Mocap Path
        self.update_path(self.mocap_path, norm_mocap_x, norm_mocap_y, mocap_yaw, current_time)
        self.mocap_path_pub.publish(self.mocap_path)

    def update_path(self, path_msg, x, y, yaw, timestamp):
        pose = PoseStamped()
        pose.header.stamp = timestamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # Simplified orientation for path trace
        path_msg.poses.append(pose)
        path_msg.header.stamp = timestamp

    def shutdown_hook(self):
        rospy.loginfo("Shutting down node and saving paths...")
        self.csv_file.close()
        # You can add the specific path saving logic here if you need separate files

    def run(self):
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()

def rotate_2d_vector(x, y, angle_rad):
    """
    Rotates a 2D vector (x, y) by angle_rad radians around the origin.
    """
    new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return new_x, new_y

if __name__ == '__main__':
    comparer = MocapComparer()
    comparer.run()