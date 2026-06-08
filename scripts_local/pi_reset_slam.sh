#!/bin/bash
# Reset SLAM map and restart clean mapping session (run on Pi).

set -e
export ROS_LOCALHOST_ONLY=1

source /opt/ros/jazzy/setup.bash
source /home/pi/ldlidar_ros2_ws/install/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/setup.bash

echo "=== stop SLAM ==="
pkill -f 'ros2 launch pupper_slam' 2>/dev/null || true
pkill -f async_slam_toolbox_node 2>/dev/null || true
sleep 2

echo "=== ensure robot running ==="
if ! systemctl is-active --quiet robot; then
  sudo systemctl start robot
  sleep 25
fi
echo "robot: $(systemctl is-active robot)"

echo "=== restart lidar ==="
pkill -f ldlidar_stl_ros2_node 2>/dev/null || true
pkill -f 'ros2 launch ldlidar_stl_ros2' 2>/dev/null || true
sleep 1
nohup ros2 launch ldlidar_stl_ros2 ld06.launch.py > /tmp/ld06.log 2>&1 &
sleep 4

echo "=== start fresh SLAM ==="
nohup ros2 launch pupper_slam slam.launch.py > /tmp/pupper_slam.log 2>&1 &
sleep 10

echo "=== verify ==="
echo -n "slam: "; pgrep -fc async_slam_toolbox_node || echo 0
ros2 lifecycle get /slam_toolbox 2>&1 || true
timeout 5 ros2 topic echo /map_metadata --once 2>&1 | grep -E 'width|height|resolution' || true
echo ""
echo "Fresh map ready. Foxglove Fixed Frame = map, then drive slowly to rebuild."
