#!/usr/bin/env bash
# EK Lab 3 commands used for the installed ROS 2 Jazzy environment.
# Run manually in separate terminals while the corresponding checkpoint is playing.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Part B
ros2 topic echo /clock --once
# Part C (Ctrl+C after several seconds, then send zero)
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}, angular: {z: 0.0}}'
# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'
# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
# Part D
ros2 topic type /scan
# ros2 topic hz /scan
ros2 topic echo /scan --once
# Part E
ros2 topic type /camera/image_raw
ros2 topic type /camera/camera_info
# ros2 topic hz /camera/image_raw
# ros2 run rqt_image_view rqt_image_view /camera/image_raw
# Part F
ros2 topic type /odom
# ros2 topic hz /odom
# ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true
# ros2 run tf2_tools view_frames -o EK_F_tf_tree
# Diagnostics
ros2 topic list -t
ros2 topic info /scan --verbose
