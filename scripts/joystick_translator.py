#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTranslator:
    def __init__(self):
        rospy.init_node('joystick_translator')

        # --- Parameters ---
        self.scale_linear = rospy.get_param('~scale_linear', 0.5)   # m/s
        self.scale_angular = rospy.get_param('~scale_angular', 1.0) # rad/s
        
        # 'arcade' (single stick) or 'tank' (dual stick)
        self.control_scheme = rospy.get_param('~control_scheme', 'arcade') 

        # Axes & Buttons (Matching your single_joystick_control.py)
        self.deadman_button = rospy.get_param('~deadman_button', 4)
        self.axis_linear = rospy.get_param('~axis_linear', 1)       # Left Stick Y
        self.axis_angular = rospy.get_param('~axis_angular', 0)     # Left Stick X
        self.axis_right_y = rospy.get_param('~axis_right_y', 4)     # Right Stick Y (Used for Tank)

        # Publishers & Subscribers
        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
        rospy.Subscriber('/my_joy', Joy, self.joy_callback)

        rospy.loginfo("Joystick Translator Active.")
        rospy.loginfo("Mode: %s | Deadman: Button %d", self.control_scheme.upper(), self.deadman_button)

    def joy_callback(self, data):
        target = Twist()

        # --- 1. Deadman Safety Check ---
        # User MUST hold the deadman button to move
        if data.buttons[self.deadman_button] == 1:

            # --- 2. Control Schemes ---
            if self.control_scheme == 'tank':
                # TANK CONTROL
                # Left track = Left Stick Y, Right track = Right Stick Y
                # Note: We apply the negative sign just like your script, because Up = -1.0
                left_input = -data.axes[self.axis_linear]
                right_input = -data.axes[self.axis_right_y]

                # Math for Differential Drive from Tank inputs
                target.linear.x = ((left_input + right_input) / 2.0) * self.scale_linear
                target.angular.z = ((right_input - left_input) / 2.0) * self.scale_angular

            else:
                # ARCADE CONTROL (Single Stick)
                # Left Stick Y = Forward/Back, Left Stick X = Turn
                # Inverted linear sign based on your controller setup
                target.linear.x = -data.axes[self.axis_linear] * self.scale_linear
                target.angular.z = data.axes[self.axis_angular] * self.scale_angular

        else:
            # Deadman NOT pressed -> Stop the robot immediately
            target.linear.x = 0.0
            target.angular.z = 0.0

        self.pub.publish(target)

if __name__ == '__main__':
    try:
        JoystickTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass