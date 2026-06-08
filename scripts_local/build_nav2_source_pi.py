"""Build Nav2 from source on the Pi when Jazzy deb packages are unavailable."""

import paramiko

HOST = "192.168.31.70"
USER = "pi"
PASSWORD = "CHANGE_ME"

NAV_WS = "$HOME/nav2_ws"
SETUP = "source /opt/ros/jazzy/setup.bash"
NAV2_TARGETS = (
    "nav2_amcl nav2_map_server nav2_controller nav2_planner "
    "nav2_bt_navigator nav2_behaviors nav2_smoother nav2_smac_planner "
    "dwb_core dwb_plugins dwb_critics costmap_queue"
)
ROSDEP_PATHS = (
    "src/BehaviorTree.CPP "
    "src/bond_core/bond "
    "src/bond_core/bondcpp "
    "src/bond_core/smclib "
    "src/common_interfaces/nav_msgs "
    "src/geographic_info/geographic_msgs "
    "src/navigation2/nav2_common "
    "src/navigation2/nav2_msgs "
    "src/navigation2/nav2_util "
    "src/navigation2/nav2_lifecycle_manager "
    "src/navigation2/nav2_core "
    "src/navigation2/nav2_voxel_grid "
    "src/navigation2/nav2_costmap_2d "
    "src/navigation2/nav2_amcl "
    "src/navigation2/nav2_map_server "
    "src/navigation2/nav2_controller "
    "src/navigation2/nav2_planner "
    "src/navigation2/nav2_smoother "
    "src/navigation2/nav2_smac_planner "
    "src/navigation2/nav2_behaviors "
    "src/navigation2/nav2_behavior_tree "
    "src/navigation2/nav2_bt_navigator "
    "src/navigation2/nav2_dwb_controller/nav_2d_msgs "
    "src/navigation2/nav2_dwb_controller/nav_2d_utils "
    "src/navigation2/nav2_dwb_controller/costmap_queue "
    "src/navigation2/nav2_dwb_controller/dwb_msgs "
    "src/navigation2/nav2_dwb_controller/dwb_core "
    "src/navigation2/nav2_dwb_controller/dwb_critics "
    "src/navigation2/nav2_dwb_controller/dwb_plugins"
)


def run(client, label, cmd, timeout=3600):
    print(f"\n=== {label} ===")
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    print(out)
    if err:
        print(err)
    print(f"[rc={rc}]")
    if rc != 0:
        raise SystemExit(rc)


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        HOST,
        username=USER,
        password=PASSWORD,
        timeout=15,
        allow_agent=False,
        look_for_keys=False,
    )
    try:
        run(
            client,
            "prepare tools",
            "bash -lc 'sudo apt update && "
            "sudo apt install -y git vcstool colcon python3-rosdep2 python3-pip "
            "python3-colcon-cmake python3-colcon-ros python3-colcon-pkg-config "
            "build-essential cmake libceres-dev libompl-dev "
            "libgraphicsmagick++-dev libboost-all-dev libsqlite3-dev "
            "libzmq3-dev libncurses-dev libreadline-dev libssl-dev "
            "nlohmann-json3-dev libeigen3-dev libxtensor-dev libxsimd-dev "
            "libbenchmark-dev libnanoflann-dev python3-transforms3d'",
            timeout=1800,
        )
        run(
            client,
            "rosdep init/update",
            "bash -lc 'sudo rosdep init 2>/dev/null || true; rosdep update --rosdistro jazzy'",
            timeout=1800,
        )
        run(
            client,
            "clone source dependencies",
            "bash -lc 'mkdir -p "
            f"{NAV_WS}/src && "
            f"if [ ! -d {NAV_WS}/src/navigation2/.git ]; then "
            f"git clone -b jazzy --depth 1 https://github.com/ros-navigation/navigation2.git {NAV_WS}/src/navigation2; "
            "else "
            f"cd {NAV_WS}/src/navigation2 && git fetch origin jazzy --depth 1 && git checkout jazzy && git pull --ff-only; "
            "fi; "
            f"if [ ! -d {NAV_WS}/src/geographic_info/.git ]; then "
            f"git clone -b ros2 --depth 1 https://github.com/ros-geographic-info/geographic_info.git {NAV_WS}/src/geographic_info; "
            "else "
            f"cd {NAV_WS}/src/geographic_info && git fetch origin ros2 --depth 1 && git checkout ros2 && git pull --ff-only; "
            "fi; "
            f"if [ ! -d {NAV_WS}/src/bond_core/.git ]; then "
            f"git clone -b jazzy --depth 1 https://github.com/ros/bond_core.git {NAV_WS}/src/bond_core; "
            "else "
            f"cd {NAV_WS}/src/bond_core && git fetch origin jazzy --depth 1 && git checkout jazzy && git pull --ff-only; "
            "fi; "
            f"if [ ! -d {NAV_WS}/src/common_interfaces/.git ]; then "
            f"git clone -b jazzy --depth 1 https://github.com/ros2/common_interfaces.git {NAV_WS}/src/common_interfaces; "
            "else "
            f"cd {NAV_WS}/src/common_interfaces && git fetch origin jazzy --depth 1 && git checkout jazzy && git pull --ff-only; "
            "fi; "
            f"if [ ! -d {NAV_WS}/src/BehaviorTree.CPP/.git ]; then "
            f"git clone -b 4.7.3 --depth 1 https://github.com/BehaviorTree/BehaviorTree.CPP.git {NAV_WS}/src/BehaviorTree.CPP; "
            "else "
            f"cd {NAV_WS}/src/BehaviorTree.CPP && "
            "git fetch origin refs/tags/4.7.3:refs/tags/4.7.3 --depth 1 && "
            "git checkout tags/4.7.3; "
            "fi'",
            timeout=1800,
        )
        run(
            client,
            "install rosdeps for nav2 subset",
            f"bash -lc '{SETUP} && cd {NAV_WS} && "
            f"rosdep install --from-paths {ROSDEP_PATHS} --ignore-src -r -y --rosdistro jazzy "
            "-t buildtool -t buildtool_export -t build -t build_export -t exec "
            "--skip-keys \"fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers "
            "ros_gz_bridge ros_gz_sim slam_toolbox nav2_minimal_tb3_sim nav2_minimal_tb4_sim "
            "tf_transformations robot_localization cv_bridge image_transport\"'",
            timeout=3600,
        )
        run(
            client,
            "patch nav2_behavior_tree for BT.CPP 4.7",
            f"bash -lc 'python3 - <<\"PY\"\n"
            f"from pathlib import Path\n"
            f"p = Path.home() / \"nav2_ws/src/navigation2/nav2_behavior_tree/src/behavior_tree_engine.cpp\"\n"
            f"lines = p.read_text().splitlines(keepends=True)\n"
            f"start = next((i for i, line in enumerate(lines) if \"BT::NodeExecutionError\" in line), None)\n"
            f"if start is None:\n"
            f"    print(\"patch already applied\", p)\n"
            f"else:\n"
            f"    end = next(i for i in range(start + 1, len(lines)) if \"catch (const std::exception\" in lines[i])\n"
            f"    patched = lines[:start] + lines[end:]\n"
            f"    p.write_text(\"\".join(patched))\n"
            f"    print(\"patched\", p)\n"
            f"PY'",
            timeout=120,
        )
        run(
            client,
            "build nav2 subset",
            f"bash -lc '{SETUP} && cd {NAV_WS} && "
            "source install/setup.bash 2>/dev/null || true; "
            "rm -rf build/behaviortree_cpp install/behaviortree_cpp log/latest_build/behaviortree_cpp "
            "build/nav2_behavior_tree build/nav2_bt_navigator "
            "build/nav_msgs build/nav2_msgs build/nav2_costmap_2d build/nav2_core build/nav2_controller "
            "build/nav2_planner build/nav2_smoother build/nav2_smac_planner "
            "build/nav2_behaviors "
            "build/dwb_core build/dwb_plugins build/dwb_critics build/costmap_queue; "
            "MAKEFLAGS=-j1 colcon build --symlink-install --parallel-workers 1 "
            f"--packages-up-to {NAV2_TARGETS} "
            "--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "
            "-DBTCPP_EXAMPLES=OFF -DBTCPP_BUILD_TOOLS=OFF'",
            timeout=14400,
        )
        run(
            client,
            "verify nav2",
            f"bash -lc '{SETUP} && source {NAV_WS}/install/setup.bash && "
            "ros2 pkg executables nav2_amcl && ros2 pkg executables nav2_controller && "
            "ros2 pkg executables nav2_planner && ros2 interface show nav2_msgs/action/NavigateToPose >/dev/null && "
            "echo NAV2_SOURCE_BUILD_OK'",
            timeout=600,
        )
    finally:
        client.close()


if __name__ == "__main__":
    main()
