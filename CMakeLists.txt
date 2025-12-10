cmake_minimum_required(VERSION 3.0.2)
project(fetch_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install the python scripts so ROS can find them
catkin_install_python(PROGRAMS
  scripts/safety_controller_twist.py
  scripts/safety_controller_runstop.py
  scripts/cmd_delay.py
  scripts/admittance_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)