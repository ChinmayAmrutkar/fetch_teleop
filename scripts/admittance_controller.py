#!/usr/bin/env python

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist

class AdmittanceController:
    def __init__(self):
        """
        Initializes the Admittance Controller node.
        - Applies a virtual mass-damper system (Physics based).
        - Includes Watchdog Timer for safety (System based).
        """
        rospy.init_node('admittance_controller')

        # --- Physics Parameters ---
        # Higher Mass = Slower acceleration (Heavier feel)
        self.VIRTUAL_MASS = rospy.get_param("~virtual_mass", 0.5)
        # Higher Damping = More resistance (Stops faster)
        self.VIRTUAL_DAMPING = rospy.get_param("~virtual_damping", 1.0)
        self.MAX_LINEAR_VEL = rospy.get_param("~max_linear_vel", 1.0)
        self.MAX_ANGULAR_VEL = rospy.get_param("~max_angular_vel", 1.5)

        # --- Safety Watchdog ---
        # If no command received for 0.5s, assume connection lost
        self.cmd_timeout = 0.5
        self.last_cmd_time = rospy.Time.now()

        # --- State Variables ---
        self.target_vel = Twist()
        self.commanded_linear_vel = 0.0
        self.commanded_angular_vel = 0.0
        self.lock = threading.Lock()
        
        self.last_loop_time = None

        # --- Publishers and Subscribers ---
        # Input: Comes from the Delay Node
        self.sub = rospy.Subscriber('/cmd_vel_delayed', Twist, self.cmd_callback)
        # Output: Goes to the Safety Node
        self.pub = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size=1)

        self.control_rate = rospy.Rate(50)
        rospy.loginfo("Admittance Controller (Mass-Spring-Damper) Initialized.")

    def cmd_callback(self, msg):
        """Thread-safe method to store the latest user intent."""
        with self.lock:
            self.last_cmd_time = rospy.Time.now()
            self.target_vel = msg

    def run(self):
        """Main control loop."""
        while not rospy.is_shutdown():
            # 1. Thread-Safe Read
            with self.lock:
                target = self.target_vel
                last_cmd_t = self.last_cmd_time

            # 2. Watchdog Check (CRITICAL SAFETY)
            # If the user stopped pressing keys, the delay node might stop publishing.
            # We must force the target to zero if the signal is stale.
            if (rospy.Time.now() - last_cmd_t).to_sec() > self.cmd_timeout:
                target.linear.x = 0.0
                target.angular.z = 0.0

            # 3. Time Delta Calculation
            now = rospy.Time.now()
            if self.last_loop_time is None:
                self.last_loop_time = now
                self.control_rate.sleep()
                continue
            
            dt = (now - self.last_loop_time).to_sec()
            self.last_loop_time = now

            if dt <= 0:
                continue

            # 4. Admittance Dynamics (F = ma + cv) => (a = (F - cv)/m)
            # We treat the user 'target' velocity as a 'Force' being applied
            force_linear = target.linear.x
            force_angular = target.angular.z

            # Calculate Acceleration
            accel_linear = (force_linear - self.VIRTUAL_DAMPING * self.commanded_linear_vel) / self.VIRTUAL_MASS
            accel_angular = (force_angular - self.VIRTUAL_DAMPING * self.commanded_angular_vel) / self.VIRTUAL_MASS

            # Integration (v = v + a*dt)
            self.commanded_linear_vel += accel_linear * dt
            self.commanded_angular_vel += accel_angular * dt

            # 5. Saturation & Cleanup
            self.commanded_linear_vel = np.clip(self.commanded_linear_vel, -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
            self.commanded_angular_vel = np.clip(self.commanded_angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

            # Auto-damp to absolute zero if inputs are zero (prevents floating point drift)
            if abs(target.linear.x) < 1e-3 and abs(self.commanded_linear_vel) < 1e-3:
                self.commanded_linear_vel = 0.0
            if abs(target.angular.z) < 1e-3 and abs(self.commanded_angular_vel) < 1e-3:
                self.commanded_angular_vel = 0.0

            # 6. Publish
            final_twist = Twist()
            final_twist.linear.x = self.commanded_linear_vel
            final_twist.angular.z = self.commanded_angular_vel

            self.pub.publish(final_twist)
            self.control_rate.sleep()

if __name__ == '__main__':
    try:
        controller = AdmittanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass