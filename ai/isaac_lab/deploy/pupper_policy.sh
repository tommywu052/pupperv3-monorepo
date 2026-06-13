#!/bin/bash
# Launcher for the Isaac Lab ONNX policy node (joystick teleop).
# Mirrors robot.sh: source ROS + workspace overlay, then run the node.
set -e
source /opt/ros/jazzy/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
# Square (button 3) = stand + drive; sticks = move. Stays in standby until pressed.
exec python3 /home/pi/pupper_policy/pupper_onnx_node.py \
    --engage --switch --joy --joy-engage-button 3
