"""Build slam_toolbox jazzy branch on Pi with bond_core."""

import sys
import time
import paramiko

HOST = "192.168.31.70"
USER = "pi"
PASSWORD = "CHANGE_ME"
WS = "/home/pi/pupperv3-monorepo/ros2_ws"
SETUP = (
    "source /opt/ros/jazzy/setup.bash && "
    f"source {WS}/install/setup.bash && "
    "source /home/pi/ldlidar_ros2_ws/install/setup.bash"
)


def run(client, cmd: str, timeout: int = 900) -> tuple[int, str]:
    _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    if err.strip():
        out += "\n--- stderr ---\n" + err
    return rc, out


def main() -> int:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASSWORD,
                   timeout=10, allow_agent=False, look_for_keys=False)

    steps = [
        ("slam jazzy branch", (
            f"rm -rf {WS}/src/slam_toolbox && "
            f"git clone -b jazzy --depth 1 "
            f"https://github.com/SteveMacenski/slam_toolbox.git {WS}/src/slam_toolbox"
        )),
        ("bond_core", (
            f"test -d {WS}/src/bond_core/.git || "
            f"git clone -b ros2 --depth 1 "
            f"https://github.com/ros/bond_core.git {WS}/src/bond_core"
        )),
        ("apt deps", (
            "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "
            "libceres-dev libsuitesparse-dev libboost-all-dev libeigen3-dev 2>&1 | tail -5"
        )),
        ("patch no-rviz", (
            f"cd {WS}/src/slam_toolbox && "
            f"python3 {WS}/src/pupper_slam/scripts/patch_slam_toolbox_no_rviz.py"
        )),
        ("colcon build", (
            f"bash -lc 'source /opt/ros/jazzy/setup.bash && "
            f"source {WS}/install/setup.bash 2>/dev/null; "
            f"cd {WS} && colcon build --packages-select bond smclib bondcpp pupper_slam slam_toolbox "
            "--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_RVIZ_PLUGIN=OFF -DBUILD_TESTING=OFF 2>&1'"
        )),
        ("verify", f"bash -lc '{SETUP} && ros2 pkg executables slam_toolbox'"),
    ]

    for name, cmd in steps:
        print(f"\n=== {name} ===")
        rc, out = run(client, cmd, timeout=1200)
        print(out[-12000:])
        print(f"exit {rc}")
        if rc != 0 and name in ("colcon build",):
            client.close()
            return rc

    # Start slam if robot running
    print("\n=== start slam ===")
    run(client, "pkill -f async_slam_toolbox_node 2>/dev/null || true")
    time.sleep(1)
    transport = client.get_transport()
    chan = transport.open_session()
    chan.exec_command(
        f"bash -lc '{SETUP} && "
        "nohup ros2 launch pupper_slam slam.launch.py > /tmp/pupper_slam.log 2>&1 &'"
    )
    time.sleep(15)

    for name, cmd in [
        ("log", "tail -25 /tmp/pupper_slam.log"),
        ("map hz", f"bash -lc '{SETUP} && timeout 10 ros2 topic hz /map 2>&1'"),
        ("topics", f"bash -lc '{SETUP} && ros2 topic list | grep -E \"^/map$|slam\"'"),
        ("tf map->odom", f"bash -lc '{SETUP} && timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -10'"),
    ]:
        print(f"\n=== {name} ===")
        rc, out = run(client, cmd, timeout=30)
        print(out[:5000])

    client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
