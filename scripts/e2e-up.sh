#!/usr/bin/env bash
# Open the Controller E2E processes from docs/07-test-scenario.md in separate
# Terminal.app windows. Run this on the macOS ROS host, not in a container.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$PROJECT_DIR/install/setup.bash"
LOGIN_SHELL="${SHELL:-/bin/zsh}"

for required in osascript docker; do
  command -v "$required" >/dev/null || {
    echo "Missing required command: $required" >&2
    exit 1
  }
done

[[ -f "$PROJECT_DIR/.env" ]] || {
  echo "Missing $PROJECT_DIR/.env" >&2
  exit 1
}
[[ -f "$ROS_SETUP" && -f "$WORKSPACE_SETUP" ]] || {
  echo "Build the workspace first: colcon build --symlink-install" >&2
  exit 1
}

open_terminal() {
  local title="$1"
  local command="$2"
  local setup

  printf -v setup 'cd %q\nsource %q\nsource %q\nexport ROS_DOMAIN_ID=%q\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nprintf "\\n===== %s =====\\n"\n%s\nstatus=$?\nprintf "\\n===== %s exited (%s); shell kept open =====\\n" "$status"\nexec %q -l' \
    "$PROJECT_DIR" "$ROS_SETUP" "$WORKSPACE_SETUP" "${ROS_DOMAIN_ID:-20}" \
    "$title" "$command" "$title" "$LOGIN_SHELL"

  osascript - "$setup" <<'APPLESCRIPT'
on run argv
  tell application "Terminal"
    activate
    do script (item 1 of argv)
  end tell
end run
APPLESCRIPT
}

cd "$PROJECT_DIR"
docker compose up -d postgres mongodb
docker compose ps postgres mongodb
docker compose up -d vision

open_terminal "MoveIt2 / M0609" 'ros2 launch dsr_moveit_config_m0609 start.launch.py mode:=real name:=dsr01 model:=m0609 gripper:=rg2 host:=192.168.1.100 gui:=true'
open_terminal "RealSense" 'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true'
open_terminal "Vision logs" 'docker compose logs -f vision'
open_terminal "DB node" 'set -a; source .env; set +a; ros2 run kit_db db_node'
open_terminal "Position estimation" 'ros2 run kit_robot position_estimation'
open_terminal "Voice command" 'ros2 run kit_voice get_command'
open_terminal "Echo command result" 'ros2 topic echo /kit/command_result'
open_terminal "Echo component result" 'ros2 topic echo /kit/component_result'
open_terminal "Echo task status" 'ros2 topic echo /kit/task_status'
open_terminal "Controller" 'ros2 run kit_robot controller --ros-args --params-file src/kit_robot/resource/controller.yaml -p restart_delay_sec:=60.0'

echo "Started Docker services and opened 10 Terminal.app windows."
