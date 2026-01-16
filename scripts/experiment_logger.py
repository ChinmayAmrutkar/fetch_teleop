#!/usr/bin/env python

import rospy
import csv
import os
import datetime
import math
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class ExperimentLogger:
    def __init__(self):
        rospy.init_node('experiment_logger')

        # --- Parameters ---
        self.max_time = rospy.get_param('~max_time', 60.0)
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        
        # --- File Setup ---
        # 1. Setup Directory
        requested_path = "/home/chinmay/fetch_teleop_ws/logs/"
        if os.path.exists(os.path.dirname(requested_path)):
            self.log_dir = requested_path
        else:
            self.log_dir = os.path.expanduser("~/fetch_teleop_ws/logs/")
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)

        # 2. Generate Base Filenames
        ts = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.files = {
            'A_mocap': os.path.join(self.log_dir, "A_mocap_{}.csv".format(ts)),
            'B_amcl': os.path.join(self.log_dir, "B_amcl_{}.csv".format(ts)),
            'C1_sync_event': os.path.join(self.log_dir, "C1_sync_event_{}.csv".format(ts)),
            'C2_sync_fixed': os.path.join(self.log_dir, "C2_sync_fixed_{}.csv".format(ts))
        }

        # 3. Initialize CSV Writers
        self.writers = {}
        self.handles = {}
        
        # Headers
        headers_common = ["Time_Sec", "Input_JX", "Input_JY", "Event_Msg"]
        
        # File A: MoCap High Freq
        self.init_csv('A_mocap', headers_common + [
            "MoCap_X", "MoCap_Y", "MoCap_Z", "MoCap_Theta",
            "Vel_Lin_Raw", "Vel_Ang_Raw",
            "Vel_Lin_Smooth", "Vel_Ang_Smooth"
        ])
        
        # File B: AMCL (Event based on AMCL update)
        self.init_csv('B_amcl', headers_common + [
            "AMCL_X", "AMCL_Y", "AMCL_Z", "AMCL_Theta",
            "Odom_Vel_Lin", "Odom_Vel_Ang" 
        ])
        
        # File C1 & C2: Synced
        headers_sync = headers_common + [
            "AMCL_X", "AMCL_Y", "AMCL_Theta",
            "MoCap_X", "MoCap_Y", "MoCap_Theta",
            "Odom_Vel_Lin", "Odom_Vel_Ang",
            "MoCap_Vel_Lin_Smooth", "MoCap_Vel_Ang_Smooth"
        ]
        self.init_csv('C1_sync_event', headers_sync)
        self.init_csv('C2_sync_fixed', headers_sync)

        # --- State Variables ---
        self.start_time = rospy.Time.now()
        self.experiment_active = True
        self.safety_triggered = False
        
        # Data Containers
        self.current_input = Twist()
        self.latest_odom_twist = Twist() # From /odom
        self.latest_amcl_pose = None # [x, y, z, theta]
        self.latest_mocap_pose = None # [x, y, z, theta]
        
        # MoCap Derivatives
        self.last_mocap_pos = None
        self.last_mocap_time = None
        self.last_mocap_theta = 0.0
        
        # Smoothing Buffers (Window Size 10)
        self.vel_lin_buffer = deque(maxlen=10)
        self.vel_ang_buffer = deque(maxlen=10)
        self.latest_mocap_vel_smooth = [0.0, 0.0] # [Lin, Ang]

        # --- Subscribers ---
        rospy.Subscriber('/cmd_vel_raw', Twist, self.input_callback)
        rospy.Subscriber('/safety_status', Bool, self.safety_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        rospy.Subscriber(mocap_topic, PoseStamped, self.mocap_callback)

        # --- Timers ---
        # Fixed Rate Logging (20Hz) for File C2
        rospy.Timer(rospy.Duration(0.05), self.fixed_rate_loop)

        rospy.loginfo("Experiment Logger Initialized. 4 Logs generating in: {}".format(self.log_dir))

    def init_csv(self, key, header):
        f = open(self.files[key], 'w')
        w = csv.writer(f)
        w.writerow(header)
        self.handles[key] = f
        self.writers[key] = w

    # --- Callbacks ---
    def input_callback(self, msg):
        self.current_input = msg

    def safety_callback(self, msg):
        self.safety_triggered = msg.data

    def odom_callback(self, msg):
        # We trust Odom for velocity of the AMCL file
        self.latest_odom_twist = msg.twist.twist

    def amcl_callback(self, msg):
        # 1. Parse AMCL
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.latest_amcl_pose = [p.x, p.y, p.z, theta]

        # 2. Write to File B (AMCL Timeseries)
        self.write_row('B_amcl', [
            self.get_time(), 
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            p.x, p.y, p.z, theta,
            self.latest_odom_twist.linear.x, self.latest_odom_twist.angular.z
        ])

        # 3. Write to File C1 (Strict Sync - Triggered by AMCL update)
        # We grab the closest available MoCap data
        if self.latest_mocap_pose is not None:
            self.write_row('C1_sync_event', self.get_sync_row())

    def mocap_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        p = msg.pose.position
        o = msg.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        
        # Calculate Velocities
        vel_lin_raw = 0.0
        vel_ang_raw = 0.0
        
        if self.last_mocap_pos is not None:
            dt = current_time - self.last_mocap_time
            if dt > 0.0001: # Avoid division by zero
                # Linear Euclidean Distance
                dist = math.sqrt((p.x - self.last_mocap_pos.x)**2 + (p.y - self.last_mocap_pos.y)**2)
                vel_lin_raw = dist / dt
                
                # Angular Shortest Path
                # Handle wrapping -pi to pi
                diff = theta - self.last_mocap_theta
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                vel_ang_raw = diff / dt

        # Update History
        self.last_mocap_pos = p
        self.last_mocap_time = current_time
        self.last_mocap_theta = theta
        self.latest_mocap_pose = [p.x, p.y, p.z, theta]

        # Smoothing (Moving Average)
        self.vel_lin_buffer.append(vel_lin_raw)
        self.vel_ang_buffer.append(vel_ang_raw)
        
        vel_lin_smooth = sum(self.vel_lin_buffer) / len(self.vel_lin_buffer)
        vel_ang_smooth = sum(self.vel_ang_buffer) / len(self.vel_ang_buffer)
        self.latest_mocap_vel_smooth = [vel_lin_smooth, vel_ang_smooth]

        # Write to File A (MoCap Timeseries)
        self.write_row('A_mocap', [
            self.get_time(),
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            p.x, p.y, p.z, theta,
            vel_lin_raw, vel_ang_raw,
            vel_lin_smooth, vel_ang_smooth
        ])

    def fixed_rate_loop(self, event):
        # Write to File C2 (Fixed Rate Sync)
        if self.latest_amcl_pose is not None and self.latest_mocap_pose is not None:
             self.write_row('C2_sync_fixed', self.get_sync_row())

    # --- Helpers ---
    def get_sync_row(self):
        # Helper to construct the synced row for C1 and C2
        # Order: Common, AMCL(4), MoCap(3), OdomVel(2), MoCapVel(2)
        return [
            self.get_time(),
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            self.latest_amcl_pose[0], self.latest_amcl_pose[1], self.latest_amcl_pose[3], # AMCL X,Y,Theta
            self.latest_mocap_pose[0], self.latest_mocap_pose[1], self.latest_mocap_pose[3], # MoCap X,Y,Theta
            self.latest_odom_twist.linear.x, self.latest_odom_twist.angular.z,
            self.latest_mocap_vel_smooth[0], self.latest_mocap_vel_smooth[1]
        ]

    def get_time(self):
        return "{:.3f}".format((rospy.Time.now() - self.start_time).to_sec())

    def get_event(self):
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if self.safety_triggered: return "SAFETY_STOP"
        if elapsed > self.max_time: return "MAX_TIME"
        return "RUNNING"

    def write_row(self, key, data):
        if not self.experiment_active: return
        self.writers[key].writerow(data)

    def shutdown_hook(self):
        # Log final manual stop event to all files
        final_row_common = [self.get_time(), 0, 0, "GOAL_REACHED_MANUAL"]
        # (Simplified logic: just closing files to ensure data flush)
        for key in self.handles:
            self.handles[key].close()
        rospy.loginfo("Experiment Logs Saved.")

if __name__ == '__main__':
    node = ExperimentLogger()
    rospy.on_shutdown(node.shutdown_hook)
    rospy.spin()