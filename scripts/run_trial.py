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
    "0": {
        "tag": "TRAIN", 
        "yaml": "figure_8_map.yaml", 
        "tf_x": 0.0, "tf_y": 0.0, "tf_yaw": 0.0
    },
    "1": {
        "tag": "EASY", 
        "yaml": "lab_map_easy.yaml", 
        "tf_x": 0.0, "tf_y": 0.0, "tf_yaw": 0.0
    },
    "2": {
        "tag": "MED", 
        "yaml": "lab_map_med.yaml", 
        "tf_x": 2.54, "tf_y": -1.2, "tf_yaw": 1.57 # Example values
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

    # 2. Control Scheme (Tank vs Separate)
    print("\nControl Schemes:")
    print("  [1] Separate / Arcade (Left Stick=Move, Right Stick=Turn)")
    print("  [2] Tank Controls (Left Stick=Left Track, Right Stick=Right Track)")
    ctrl_choice = input_fn("Select Control [1/2] [Default: 1]: ").strip()
    scheme_str = "tank" if ctrl_choice == "2" else "arcade"
    ctrl_tag = "TANK" if scheme_str == "tank" else "ARCADE"

    # 3. Select Map
    print("\nAvailable Maps:")
    for key in sorted(MAP_DATABASE.keys()):
        print("  [{}] {} ({})".format(key, MAP_DATABASE[key]['tag'], MAP_DATABASE[key]['yaml']))
    
    map_choice = input_fn("Select Map (0/1/2/3) [Default: 2]: ").strip()
    if map_choice not in MAP_DATABASE: map_choice = "2"
    sel_map = MAP_DATABASE[map_choice]
    
    is_training = (sel_map['tag'] == "TRAIN")

    # 4. ONE-WAY Delay
    print("\nDelay Configuration:")
    delay_input = input_fn("Enter ONE-WAY delay in seconds (0.0, 0.25, 0.5, 0.75) [Default: 0.25]: ").strip()
    try:
        one_way_delay = float(delay_input)
    except:
        one_way_delay = 0.25
    
    # 5. Rigid Body Name
    rb_name = input_fn("\nEnter MoCap Rigid Body Name [Default: Fetch8]: ").strip()
    if not rb_name: rb_name = "Fetch8"

    # Generate File Prefix
    log_prefix = "{}_{}_{}_Delay{:.2f}".format(pid.upper(), ctrl_tag, sel_map['tag'], one_way_delay)

    # --- NEW: Map the Tag to the layout name required by camera_client ---
    # We map TRAIN to "easy" so the camera loads an image and doesn't crash 
    # looking for a non-existent map_train.png
    layout_mapping = {"TRAIN": "easy", "EASY": "easy", "MED": "medium", "HARD": "hard"}
    layout_str = layout_mapping.get(sel_map['tag'], "medium")

    print_header("SUMMARY")
    print("Participant:     " + pid.upper())
    print("Control Scheme:  " + ctrl_tag)
    print("Log Prefix:      " + log_prefix)
    print("Map File:        " + sel_map['yaml'])
    print("Rigid Body:      " + rb_name)
    print("One-Way Latency: {:.2f}s".format(one_way_delay))
    print("Total RTT:       {:.2f}s (Round Trip)".format(one_way_delay * 2.0))
    print("Camera Layout:   " + layout_str)
    
    if is_training:
        print("\n*** TRAINING MODE: Goal Supervisor is DISABLED ***")
    
    input_fn("\n>>> Press ENTER to launch the experiment! <<<")

    # Force the control scheme param into the ROS parameter server before launch
    # This ensures the joystick translator starts in the correct mode
    os.system("rosparam set /joystick_translator/control_scheme {}".format(scheme_str))

    # --- BUILD COMMANDS ---
    cmd_loc = "roslaunch fetch_teleop compare_localization.launch map_file:=/home/fetchuser/chinmay/fetch_teleop_ws/my_maps/{} tf_x:={} tf_y:={} tf_yaw:={} rigid_body:={} log_prefix:={}".format(
        sel_map['yaml'], sel_map['tf_x'], sel_map['tf_y'], sel_map['tf_yaw'], rb_name, log_prefix
    )

    cmd_teleop = "roslaunch fetch_teleop safety.launch method:=delayed input_device:=joystick delay_time:={} camera_ip:=192.168.0.172 layout:={}".format(
        one_way_delay, layout_str
    )

    # Only build the supervisor command if we are doing an actual trial
    if not is_training:
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

    if not is_training:
        print("\nExperiment running! Launching Supervisor in this window...")
        print("="*60)
        # The supervisor takes over the current terminal so you can hit Enter for the snapshot
        os.system(cmd_sup)
    else:
        print("\n" + "="*60)
        print("  TRAINING MODE ACTIVE")
        print("="*60)
        print("Participant is free to drive the Figure 8.")
        print("No Goal Snapshot is required.")
        print("(Note: Any background logs are prefixed with 'TRAIN' and can be safely deleted later).")
        print("\n>>> Press Ctrl+C when they have finished training. <<<")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nTraining complete. Exiting Wizard...")

if __name__ == "__main__":
    main()