#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time

# Handle Python 2 / 3 input compatibility
try: input_fn = raw_input
except NameError: input_fn = input

def print_header(title):
    print("\n" + "="*60)
    print("  " + title)
    print("="*60)

# =========================================================
# MAP CALIBRATION DATABASE
# Update these values after running calibrate_frames.py once!
# =========================================================
MAP_DATABASE = {
    "1": {
        "tag": "EASY", 
        "yaml": "lab_map_easy.yaml", 
        "tf_x": 0.0, "tf_y": 0.0, "tf_yaw": 0.0
    },
    "2": {
        "tag": "MED", 
        "yaml": "lab_map_med.yaml", 
        "tf_x": 1.1447, "tf_y": 3.9805, "tf_yaw": 1.57 # Example values
    },
    "3": {
        "tag": "HARD", 
        "yaml": "lab_map_hard.yaml", 
        "tf_x": -1.0, "tf_y": 3.2, "tf_yaw": -0.78 # Example values
    }
}

def main():
    print_header("FETCH HRI EXPERIMENT SETUP WIZARD")

    # 1. Participant ID
    pid = input_fn("Participant ID (e.g., P01): ").strip()
    if not pid: pid = "P00"

    # 2. Select Map
    print("\nAvailable Maps:")
    for key, data in MAP_DATABASE.items():
        print("  [{}] {} ({})".format(key, data['tag'], data['yaml']))
    
    map_choice = input_fn("Select Map (1/2/3) [Default: 2]: ").strip()
    if map_choice not in MAP_DATABASE: map_choice = "2"
    sel_map = MAP_DATABASE[map_choice]

    # 3. Total Delay
    print("\nNOTE: This script will automatically split the delay between Input and Video.")
    delay_input = input_fn("Enter TOTAL desired round-trip delay (e.g. 0.5, 0.75, 1.0) [Default: 0.5]: ").strip()
    try:
        total_delay = float(delay_input)
    except:
        total_delay = 0.5
    
    # MATH: Divide by 2 because delay_time applies to both controls AND camera.
    split_delay = total_delay / 2.0 

    # 4. Rigid Body Name
    rb_name = input_fn("\nEnter MoCap Rigid Body Name [Default: Fetch8]: ").strip()
    if not rb_name: rb_name = "Fetch8"

    # Generate File Prefix
    log_prefix = "{}_{}_Delay{:.2f}".format(pid.upper(), sel_map['tag'], total_delay)

    print_header("SUMMARY")
    print("Log Prefix:      " + log_prefix)
    print("Map File:        " + sel_map['yaml'])
    print("Rigid Body:      " + rb_name)
    print("Total Latency:   {:.2f}s".format(total_delay))
    print("Launch Delay Arg:{:.3f}s (Applied to both Cmd & Video)".format(split_delay))
    
    input_fn("\n>>> Press ENTER to launch the experiment! <<<")

    # --- BUILD COMMANDS ---
    cmd_loc = "roslaunch fetch_teleop compare_localization.launch map_file:=/home/fetchuser/chinmay/fetch_teleop_ws/my_maps/{} tf_x:={} tf_y:={} tf_yaw:={} rigid_body:={} log_prefix:={}".format(
        sel_map['yaml'], sel_map['tf_x'], sel_map['tf_y'], sel_map['tf_yaw'], rb_name, log_prefix
    )

    cmd_teleop = "roslaunch fetch_teleop safety.launch method:=delayed input_device:=joystick delay_time:={} camera_ip:=192.168.0.172".format(
        split_delay
    )

    cmd_sup = "rosrun fetch_teleop experiment_supervisor.py _robot_name:={} _goal_name:=Goal".format(
        rb_name
    )

    # --- LAUNCH TERMINALS ---
    # We use gnome-terminal to pop open nice, organized windows for the background processes
    print("\nStarting Localization & Logger...")
    os.system("gnome-terminal --title='Localization' -- bash -c '{}; exec bash'".format(cmd_loc))
    time.sleep(4) # Wait for AMCL to boot up

    print("Starting Control & Camera...")
    os.system("gnome-terminal --title='Controls & Video' -- bash -c '{}; exec bash'".format(cmd_teleop))
    time.sleep(3) # Wait for Video to connect

    print("\nExperiment running! Launching Supervisor in this window...")
    print("="*60)
    
    # The supervisor takes over the current terminal so you can hit Enter for the snapshot
    os.system(cmd_sup)

if __name__ == "__main__":
    main()