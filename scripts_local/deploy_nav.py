"""Deploy pupper_nav, mux config, map, and helper scripts to the Pi."""

import os
import stat

import paramiko

HOST = "192.168.31.70"
USER = "pi"
PASSWORD = "CHANGE_ME"

ROOT = r"C:\Nvidia\pupperv3"
LOCAL_SRC = os.path.join(ROOT, "pupperv3-monorepo", "ros2_ws", "src")
REMOTE_REPO = "/home/pi/pupperv3-monorepo"
REMOTE_WS = f"{REMOTE_REPO}/ros2_ws"


def run(client, cmd, timeout=600):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    if out:
        print(out)
    if err:
        print(err)
    if rc != 0:
        raise RuntimeError(f"command failed rc={rc}: {cmd}")
    return out


def mkdir_p(sftp, path):
    parts = []
    while path not in ("", "/"):
        parts.append(path)
        path = os.path.dirname(path)
    for item in reversed(parts):
        try:
            sftp.stat(item)
        except FileNotFoundError:
            sftp.mkdir(item)


def upload_tree(sftp, local_dir, remote_dir):
    mkdir_p(sftp, remote_dir)
    for root, dirs, files in os.walk(local_dir):
        dirs[:] = [d for d in dirs if d not in {".git", "__pycache__"}]
        rel = os.path.relpath(root, local_dir)
        remote_root = remote_dir if rel == "." else f"{remote_dir}/{rel.replace(os.sep, '/')}"
        mkdir_p(sftp, remote_root)
        for name in files:
            if name.endswith((".pyc", ".pyo")):
                continue
            sftp.put(os.path.join(root, name), f"{remote_root}/{name}")


def upload_file(sftp, local_path, remote_path, executable=False):
    mkdir_p(sftp, os.path.dirname(remote_path))
    sftp.put(local_path, remote_path)
    if executable:
        mode = sftp.stat(remote_path).st_mode
        sftp.chmod(remote_path, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


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
        sftp = client.open_sftp()
        upload_tree(sftp, os.path.join(LOCAL_SRC, "pupper_nav"), f"{REMOTE_WS}/src/pupper_nav")
        upload_file(
            sftp,
            os.path.join(LOCAL_SRC, "neural_controller", "launch", "config.yaml"),
            f"{REMOTE_WS}/src/neural_controller/launch/config.yaml",
        )
        upload_file(
            sftp,
            os.path.join(ROOT, "maps", "pupper_map_ekf_v1.yaml"),
            "/home/pi/maps/pupper_map_ekf_v1.yaml",
        )
        upload_file(
            sftp,
            os.path.join(ROOT, "maps", "pupper_map_ekf_v1.pgm"),
            "/home/pi/maps/pupper_map_ekf_v1.pgm",
        )
        for script in [
            "pi_start_nav.sh",
            "pi_nav_status.sh",
            "pi_nav_initialpose.sh",
            "pi_nav_goal.sh",
        ]:
            upload_file(
                sftp,
                os.path.join(ROOT, "scripts_local", script),
                f"{REMOTE_REPO}/scripts_local/{script}",
                executable=True,
            )
        sftp.close()

        setup = "source /opt/ros/jazzy/setup.bash"
        nav_setup = "source $HOME/nav2_ws/install/setup.bash 2>/dev/null || true"
        run(
            client,
            f"bash -lc '{setup} && {nav_setup} && cd {REMOTE_WS} && "
            "colcon build --packages-select pupper_nav cmd_vel_mux neural_controller --symlink-install 2>&1'",
            timeout=1800,
        )
        run(
            client,
            f"bash -lc '{setup} && source {REMOTE_WS}/install/setup.bash && "
            "ros2 pkg prefix pupper_nav && grep -A8 \"cmd_vel_mux:\" "
            f"{REMOTE_WS}/install/neural_controller/share/neural_controller/launch/config.yaml'",
            timeout=120,
        )
        print("DEPLOY_NAV_OK")
    finally:
        client.close()


if __name__ == "__main__":
    main()
