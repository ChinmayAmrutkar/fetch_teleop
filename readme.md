# Fetch Robot Teleop Framework

This package implements a robust teleoperation and research framework for the Fetch Robot. It includes safety controllers, network latency simulation, admittance control, and tools for validating localization (AMCL vs Optitrack).

## Installation

1. Clone into your workspace:
```
    cd ~/fetch_teleop_ws/src
    git clone https://github.com/ChinmayAmrutkar/fetch_teleop.git
```
2. Build the workspace:
```
    cd ~/fetch_teleop_ws
    catkin_make
    source devel/setup.bash
```
## Part 1: Safety Controllers

We implement two distinct architectures to prevent collisions.

### 1. Method A: Twist Filter (The "Soft" Stop)

* **Logic:** Intercepts `/cmd_vel` commands. If the LIDAR detects an obstacle < 0.6m, it overrides the linear velocity to `0.0` before sending it to the robot base.
* **Behavior:** Smooth stop. The robot refuses to move forward but allows rotation and backing up.
* **Best for:** Standard teleoperation assistance.

### 2. Method B: Runstop Trigger (The "Hard" Stop)

* **Logic:** Monitors LIDAR independently. If unsafe, it publishes `True` to `/enable_software_runstop`, triggering the robot's internal software E-Stop.
* **Behavior:** Immediate, abrupt stop. Motors may disengage.
* **Best for:** Emergency stop testing.

**Usage:**
```
    # Soft Stop
    roslaunch fetch_teleop safety.launch method:=twist

    # Hard Stop
    roslaunch fetch_teleop safety.launch method:=runstop
```
## 📡 Part 2: Latency & Dynamics (The "Delayed" Experiment)

This mode simulates controlling the robot over a high-latency network (e.g., Earth to Moon).

**Pipeline:**
`Keyboard` -> [Delay Node (1.0s)] -> [Admittance Controller] -> [Safety Controller] -> `Robot`

* **Delay Node:** Buffers inputs for 1 second to simulate lag.
* **Admittance Controller:** Smooths out the delayed signals (inertia simulation) to prevent jerky motion.
* **Safety Controller:** Runs in **Real-Time** at the end of the chain to guarantee safety regardless of input lag.

**Usage:**
```
    roslaunch fetch_teleop safety.launch method:=delayed
```
## Part 3: Mapping

Tools to generate a map of the lab for localization.

**1. Start the Mapper:**
```
    roslaunch fetch_teleop build_map.launch
```
*(Drive the robot around until the map looks complete in RViz)*

**2. Save the Map:**
```
    rosrun fetch_teleop save_map.py
```
* Saves to: `~/my_maps/lab_map_YYYY-MM-DD...yaml`

## Part 4: Localization Validation (AMCL vs MoCap)

Tools to compare the robot's estimated position (AMCL) against Ground Truth (Optitrack/VRPN) and log the error.

**1. Launch the Experiment:**
You need to specify your map file and the Optitrack Server IP.
```
    roslaunch fetch_teleop compare_localization.launch \
        map_file:=/home/fetch/my_maps/lab_map_YOUR_DATE.yaml \
        mocap_ip:=192.168.1.xxx
```
**2. Drive & Log:**

* In a separate terminal, run the teleop (Part 1 or Part 2).
* The system will automatically log position data to `~/pose_logs/`.

**3. Data Analysis:**

* Check the generated CSV file for columns: `Timestamp`, `AMCL_X/Y`, `MoCap_X/Y`, `Error_Distance`.
