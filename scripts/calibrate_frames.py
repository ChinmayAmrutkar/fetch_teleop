#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class FrameCalibrator:
    def __init__(self):
        rospy.init_node('frame_calibrator')

        # --- CONFIGURATION ---
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        
        self.amcl_pose = None
        self.mocap_pose = None

        # --- SUBSCRIBERS ---
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)

        rospy.loginfo("Calibrator Waiting for poses...")
        rospy.loginfo("1. Drive robot to a clear area.")
        rospy.loginfo("2. Initialize AMCL (2D Pose Estimate) so it is stable.")
        rospy.loginfo("3. Wait for calculation below...")

    def amcl_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.amcl_pose = {'x': p.x, 'y': p.y, 'z': p.z, 'theta': theta}
        self.calculate_transform()

    def mocap_callback(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.mocap_pose = {'x': p.x, 'y': p.y, 'z': p.z, 'theta': theta}
        self.calculate_transform()

    def calculate_transform(self):
        if self.amcl_pose is None or self.mocap_pose is None:
            return

        # --- THE MATH ---
        # Goal: Find the Transform T that moves Map frame to World frame.
        # 1. Rotation Offset (How much is the Map rotated relative to World?)
        yaw_offset = self.mocap_pose['theta'] - self.amcl_pose['theta']

        # 2. Translation Offset (Where is the Map Origin relative to World Origin?)
        # Formula: Origin_World = Robot_World - (Rotate(Robot_Map))
        
        # Rotate the AMCL vector by the yaw offset
        amcl_x_rot = self.amcl_pose['x'] * math.cos(yaw_offset) - self.amcl_pose['y'] * math.sin(yaw_offset)
        amcl_y_rot = self.amcl_pose['x'] * math.sin(yaw_offset) + self.amcl_pose['y'] * math.cos(yaw_offset)

        # Calculate shift
        x_shift = self.mocap_pose['x'] - amcl_x_rot
        y_shift = self.mocap_pose['y'] - amcl_y_rot
        z_shift = self.mocap_pose['z'] - self.amcl_pose['z']

        # --- OUTPUT ---
        print("\n" + "="*60)
        print("CALIBRATION CALCULATED!")
        print("="*60)
        print("Use these values in your launch file args:")
        print("tf_x: {:.4f}".format(x_shift))
        print("tf_y: {:.4f}".format(y_shift))
        print("tf_z: {:.4f}".format(z_shift))
        print("tf_yaw: {:.4f}".format(yaw_offset))
        print("-" * 60)
        
        # Print the exact XML line for copy-pasting if preferred
        print("XML Line:")
        print('<node pkg="tf" type="static_transform_publisher" name="world_map_linker" args="{:.4f} {:.4f} {:.4f} {:.4f} 0 0 world map 100" />'.format(
            x_shift, y_shift, z_shift, yaw_offset
        ))
        print("="*60 + "\n")
        
        # We only need one good calculation, then we can exit
        rospy.signal_shutdown("Calibration Done")

if __name__ == '__main__':
    FrameCalibrator()
    rospy.spin()