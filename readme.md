# Fetch Robot Safety Teleop

Compare two safety architectures for the Fetch Robot.

## The Two Methods

1.  **Method A: Twist Filter (The "Soft" Stop)**
    * **How it works:** It sits between your keyboard and the robot. When it sees an obstacle, it mathematically changes your command from `0.5 m/s` to `0.0 m/s`.
    * **Behavior:** The robot will slow down or stop smoothly. If you hold "Forward", it simply refuses to send the command.
    * **Best for:** Navigation assistance and collision avoidance.

2.  **Method B: Runstop Trigger (The "Hard" Stop)**
    * **How it works:** It watches the world. If unsafe, it sends a `True` signal to `/enable_software_runstop`. It does **not** touch your velocity commands.
    * **Behavior:** The robot motors should cut out immediately.
    * **The Risk:** If sensor noise causes the Runstop to flicker (True->False->True), and you are still holding "Forward", the robot may jerk violently as the motors re-engage at full speed.
    * **Best for:** Emergency situations (E-Stop).

## Installation
1.  Clone into `~/fetch_teleop_ws/src`
2.  `catkin_make`
3.  `source devel/setup.bash`

## Usage (The Experiment)

**Test Method A (Twist Filter):**
```bash
roslaunch fetch_teleop_safety safety.launch method:=twist
```

**Test Method B (Pure Runstop):**
```bash
roslaunch fetch_teleop_safety safety.launch method:=runstop
```

**Verify the Runstop Topic: In a separate terminal, watch the flag switch:**
```bash
rostopic echo /enable_software_runstop
```
