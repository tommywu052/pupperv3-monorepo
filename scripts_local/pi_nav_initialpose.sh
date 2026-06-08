#!/usr/bin/env bash
set -eo pipefail

X="${1:-0.0}"
Y="${2:-0.0}"
YAW="${3:-0.0}"

source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/nav2_ws/install/setup.bash" ]]; then
  source "$HOME/nav2_ws/install/setup.bash"
fi
source "$HOME/pupperv3-monorepo/ros2_ws/install/setup.bash"

MSG="$(python3 - "$X" "$Y" "$YAW" <<'PY'
import math
import sys
x = float(sys.argv[1])
y = float(sys.argv[2])
yaw = float(sys.argv[3])
z = math.sin(yaw / 2.0)
w = math.cos(yaw / 2.0)
print(
    "{header: {frame_id: map}, pose: {pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
    "orientation: {x: 0.0, y: 0.0, z: %.9f, w: %.9f}}, covariance: "
    "[0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "
    "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]}}"
    % (x, y, z, w)
)
PY
)"

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$MSG"
