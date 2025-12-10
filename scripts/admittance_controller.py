#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist

class AdmittanceController:
    def __init__(self):
        rospy.init_node('admittance_controller')

        # --- Parameters ---
        # The maximum acceleration (m/s^2). Lower = Heavier/Smoother.
        self.accel_limit = 0.5 
        # The maximum angular acceleration (rad/s^2).
        self.ang_accel_limit = 1.0
        
        # Rate of control loop (Hz)
        self.rate = 50.0
        self.dt = 1.0 / self.rate

        # State Variables (Current Velocity of the "Virtual Mass")
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # Target Variables (Input from Delay Node)
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        # --- Subscribers & Publishers ---
        # Listen to the DELAYED output
        self.sub = rospy.Subscriber('/cmd_vel_delayed', Twist, self.cmd_callback)
        
        # Publish to the topic the Safety Controller listens to
        self.pub = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=10)

        # Control Loop Timer
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_loop)

        rospy.loginfo("Admittance Controller Initialized. Accel Limit: {}".format(self.accel_limit))

    def cmd_callback(self, msg):
        # Just update the target. The timer loop handles the physics.
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

    def update_loop(self, event):
        # Calculate the difference between Target and Current
        diff_x = self.target_linear_x - self.current_linear_x
        diff_ang = self.target_angular_z - self.current_angular_z

        # --- PHYSICS LOGIC (Ramping) ---
        # Clamp the change to the maximum acceleration allowed per timestep
        max_change_x = self.accel_limit * self.dt
        max_change_ang = self.ang_accel_limit * self.dt

        # Update Linear Velocity
        if abs(diff_x) < max_change_x:
            self.current_linear_x = self.target_linear_x # Close enough, snap to target
        else:
            self.current_linear_x += np.sign(diff_x) * max_change_x

        # Update Angular Velocity
        if abs(diff_ang) < max_change_ang:
            self.current_angular_z = self.target_angular_z
        else:
            self.current_angular_z += np.sign(diff_ang) * max_change_ang

        # --- Publish ---
        out_msg = Twist()
        out_msg.linear.x = self.current_linear_x
        out_msg.angular.z = self.current_angular_z
        self.pub.publish(out_msg)

if __name__ == '__main__':
    try:
        AdmittanceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass