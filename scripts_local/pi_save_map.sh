#!/bin/bash
# Save current SLAM map for Nav2 (.pgm + .yaml).
# slam_toolbox save_map needs nav2_map_server (not on Pi) — use Python saver instead.
#
# Usage:
#   bash pi_save_map.sh
#   bash pi_save_map.sh my_room

set -e
export ROS_LOCALHOST_ONLY=1

NAME="${1:-pupper_map}"
MAP_DIR="${2:-/home/pi/maps}"
FULL="${MAP_DIR}/${NAME}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAVER="${SCRIPT_DIR}/map_saver_from_topic.py"

source /opt/ros/jazzy/setup.bash
source /home/pi/pupperv3-monorepo/ros2_ws/install/setup.bash
mkdir -p "$MAP_DIR"

echo "=== pause new scans (optional) ==="
ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause "{data: true}" 2>/dev/null || true
sleep 1

echo "=== saving map to ${FULL} ==="
python3 "$SAVER" "$FULL"

echo "=== done ==="
ls -la "${FULL}.yaml" "${FULL}.pgm"
echo ""
echo "Nav2 map_server yaml_filename: ${FULL}.yaml"
