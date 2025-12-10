#!/usr/bin/env python

import rospy
import subprocess
import os
import datetime

def save_map():
    rospy.init_node('map_saver_tool')
    
    # 1. Define storage location
    # We save to the user's home directory under 'my_maps'
    map_dir = os.path.expanduser("~/chinmay/fetch_teleop_ws/my_maps")
    if not os.path.exists(map_dir):
        os.makedirs(map_dir)
        
    # 2. Generate filename with timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    map_name = "lab_map_{}".format(timestamp)
    full_path = os.path.join(map_dir, map_name)
    
    rospy.loginfo("Saving map to: {}".format(full_path))
    
    # 3. Call the ROS map_saver CLI tool
    try:
        subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", full_path])
        rospy.loginfo("Map saved successfully!")
    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to save map: {}".format(e))

if __name__ == '__main__':
    save_map()
