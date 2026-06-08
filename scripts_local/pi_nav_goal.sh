#!/usr/bin/env bash
set -eo pipefail

X="${1:?usage: pi_nav_goal.sh X Y YAW_RADIANS}"
Y="${2:?usage: pi_nav_goal.sh X Y YAW_RADIANS}"
YAW="${3:?usage: pi_nav_goal.sh X Y YAW_RADIANS}"

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
    "{pose: {header: {frame_id: map}, pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
    "orientation: {x: 0.0, y: 0.0, z: %.9f, w: %.9f}}}}"
    % (x, y, z, w)
)
PY
)"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$MSG" --feedback
