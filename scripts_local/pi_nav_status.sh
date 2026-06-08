#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/nav2_ws/install/setup.bash" ]]; then
  source "$HOME/nav2_ws/install/setup.bash"
fi
source "$HOME/pupperv3-monorepo/ros2_ws/install/setup.bash"

echo "=== Nav2 packages ==="
ros2 pkg list | grep -E '^(nav2_|dwb_|nav_2d|behaviortree_cpp|bond)' || true

echo
echo "=== Nav nodes ==="
pgrep -af 'map_server|amcl|controller_server|planner_server|bt_navigator|lifecycle_manager_navigation' || true

echo
echo "=== Core topics ==="
ros2 topic list | grep -E '^/(map|scan|odom|tf|amcl_pose|particle_cloud|plan|nav_cmd_vel)$' || true

echo
echo "=== Lifecycle ==="
for node in map_server amcl planner_server controller_server bt_navigator; do
  echo -n "$node: "
  timeout 3 ros2 lifecycle get "/$node" 2>&1 || true
done

echo
echo "=== TF checks ==="
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8 || true
timeout 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -8 || true
