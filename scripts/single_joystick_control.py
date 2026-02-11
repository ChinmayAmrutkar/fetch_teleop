#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickTranslator:
    def __init__(self):
        rospy.init_node('joystick_translator')

        # --- CONFIGURATION (Single Stick Control) ---
        # Using LEFT stick by default:
        # Axis 1 = Up/Down  (Forward/Backward)
        # Axis 0 = Left/Right (Turning)
        self.axis_linear = rospy.get_param('~axis_linear', 1)
        self.axis_angular = rospy.get_param('~axis_angular', 0)
        
        # Keeping Deadman SAME as your original code
        self.deadman_button = rospy.get_param('~deadman_button', 4)

        # Scales
        self.scale_linear = rospy.get_param('~scale_linear', 0.5)   # m/s
        self.scale_angular = rospy.get_param('~scale_angular', 1.0) # rad/s

        # Publisher
        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)

        # Subscriber
        rospy.Subscriber("my_joy", Joy, self.callback)

        rospy.loginfo("Joystick Translator Started (Single Stick Mode)")
        rospy.loginfo("Deadman: Button %d | Linear: Axis %d | Angular: Axis %d",
                      self.deadman_button, self.axis_linear, self.axis_angular)

    def callback(self, data):
        twist = Twist()

        # Deadman safety check
        if data.buttons[self.deadman_button] == 1:

            # Forward/Backward
            # In many controllers: Up = -1.0 → so invert sign
            twist.linear.x = -data.axes[self.axis_linear] * self.scale_linear

            # Left/Right turning
            twist.angular.z = data.axes[self.axis_angular] * self.scale_angular

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)


if __name__ == '__main__':
    JoystickTranslator()
    rospy.spin()
