#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickTranslator:
    def __init__(self):
        rospy.init_node('joystick_translator')

        # --- CONFIGURATION (Default: Xbox 360 / ONN Layout) ---
        # Axes
        self.axis_linear = rospy.get_param('~axis_linear', 4)  # Right Stick Up/Down
        self.axis_angular = rospy.get_param('~axis_angular', 0) # Left Stick Left/Right
        
        # Buttons
        self.deadman_button = rospy.get_param('~deadman_button', 4) # LB (Left Bumper)

        # Scales (Max Speed)
        self.scale_linear = rospy.get_param('~scale_linear', 0.5) # m/s
        self.scale_angular = rospy.get_param('~scale_angular', 1.0) # rad/s

        # --- PUBLISHER ---
        # We publish to raw so the Delay/Logger nodes pick it up
        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)

        # --- SUBSCRIBER ---
        rospy.Subscriber("my_joy", Joy, self.callback)

        rospy.loginfo("Joystick Translator Started.")
        rospy.loginfo("Deadman: Button %d | Linear: Axis %d | Angular: Axis %d", 
                      self.deadman_button, self.axis_linear, self.axis_angular)

    def callback(self, data):
        twist = Twist()

        # 1. Check Deadman Switch (Safety)
        # If button is NOT pressed, we send 0.0 velocity
        if data.buttons[self.deadman_button] == 1:
            
            # 2. Map Axes
            # Note: Vertical axes are often 1.0 (Up) to -1.0 (Down).
            # We usually want Up to be Forward (+x), so checks signs.
            twist.linear.x = data.axes[self.axis_linear] * self.scale_linear
            twist.angular.z = data.axes[self.axis_angular] * self.scale_angular

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

if __name__ == '__main__':
    JoystickTranslator()
    rospy.spin()