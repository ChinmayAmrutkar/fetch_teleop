#!/usr/bin/env python

import rospy
import sys
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class ExperimentSupervisor:
    def __init__(self):
        rospy.init_node('experiment_supervisor')

        # --- CONFIGURATION ---
        self.robot_name = rospy.get_param('~robot_name', 'Fetch')
        self.goal_name = rospy.get_param('~goal_name', 'Goal')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3)
        
        # Obstacles: Comma-separated string from launch file (e.g., "Box1,Box2")
        obs_param = rospy.get_param('~obstacle_names', 'Box1,Box2')
        self.obstacle_names = [x.strip() for x in obs_param.split(',')]

        # --- STATE ---
        self.latest_poses = {} # Dictionary to store live poses of all objects
        self.saved_goal = None # (x, y) tuple
        self.saved_obstacles = {} # {name: (x,y)} dict
        self.snapshot_taken = False

        # --- PUBLISHERS ---
        self.goal_pub = rospy.Publisher('/goal_reached', Bool, queue_size=1)

        # --- SUBSCRIBERS ---
        # 1. Robot
        self.setup_subscriber(self.robot_name)
        # 2. Goal
        self.setup_subscriber(self.goal_name)
        # 3. Obstacles
        for name in self.obstacle_names:
            self.setup_subscriber(name)

        rospy.loginfo("Supervisor Ready. Tracking: Robot='{}', Goal='{}', Obstacles={}".format(
            self.robot_name, self.goal_name, self.obstacle_names))

    def setup_subscriber(self, body_name):
        topic = '/vrpn_client_node/{}/pose'.format(body_name)
        # We use a closure (lambda) or partial to capture the body_name
        rospy.Subscriber(topic, PoseStamped, self.pose_callback, body_name)

    def pose_callback(self, msg, body_name):
        # Update the live position
        self.latest_poses[body_name] = msg.pose.position

    def wait_for_snapshot(self):
        print("\n" + "="*40)
        print(" EXPERIMENT SETUP PHASE")
        print("="*40)
        print("1. Place the '{}' marker at the target location.".format(self.goal_name))
        print("2. Place obstacles ({}) if needed.".format(self.obstacle_names))
        print("3. Ensure Robot '{}' is visible.".format(self.robot_name))
        print("-" * 40)
        
        # Wait for user input (Python 2/3 compatible)
        try:
            input(">>> PRESS [ENTER] TO TAKE SNAPSHOT <<<")
        except SyntaxError:
            pass # Python 2 input() workaround

        # --- TAKE SNAPSHOT ---
        if self.goal_name not in self.latest_poses:
            rospy.logerr("CANNOT TAKE SNAPSHOT: Goal '{}' not visible in MoCap!".format(self.goal_name))
            return False
        
        # Save Goal
        g = self.latest_poses[self.goal_name]
        self.saved_goal = (g.x, g.y)
        rospy.loginfo("SNAPSHOT SAVED: Goal Location @ x={:.2f}, y={:.2f}".format(g.x, g.y))

        # Save Obstacles (Optional, just for logging/print)
        for obs in self.obstacle_names:
            if obs in self.latest_poses:
                p = self.latest_poses[obs]
                self.saved_obstacles[obs] = (p.x, p.y)
                rospy.loginfo("Obstacle '{}' detected @ {:.2f}, {:.2f}".format(obs, p.x, p.y))
            else:
                rospy.logwarn("Obstacle '{}' NOT detected during snapshot.".format(obs))

        print("="*40)
        print("SNAPSHOT COMPLETE. You may remove the Goal marker now.")
        print("Monitoring for Goal Reach (< {:.2f}m)...".format(self.goal_tolerance))
        print("="*40)
        
        self.snapshot_taken = True
        return True

    def run_monitor(self):
        rate = rospy.Rate(10) # 10 Hz
        
        while not rospy.is_shutdown():
            if not self.snapshot_taken:
                # If snapshot failed or skipped, don't publish logic
                rate.sleep()
                continue

            # Check Robot Distance
            if self.robot_name in self.latest_poses:
                r = self.latest_poses[self.robot_name]
                gx, gy = self.saved_goal
                
                distance = math.sqrt((r.x - gx)**2 + (r.y - gy)**2)
                
                # Logic
                reached = distance < self.goal_tolerance
                
                # Publish
                self.goal_pub.publish(reached)
                
                if reached:
                    rospy.loginfo_throttle(2, ">>> GOAL REACHED! (Dist: {:.2f}m) <<<".format(distance))
            else:
                rospy.logwarn_throttle(5, "Robot '{}' lost in MoCap!".format(self.robot_name))

            rate.sleep()

if __name__ == '__main__':
    sup = ExperimentSupervisor()
    # Phase 1: Block until user presses Enter
    if sup.wait_for_snapshot():
        # Phase 2: Loop forever checking distance
        sup.run_monitor()