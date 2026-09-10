#!/usr/bin/env bash
# Ubuntu 24.04: docs/07 Controller E2E in one 3x3 Terminator window.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$PROJECT_DIR/install/setup.bash"

for required in terminator docker uuidgen; do
  command -v "$required" >/dev/null || { echo "Missing: $required" >&2; exit 1; }
done
[[ -f "$PROJECT_DIR/.env" ]] || { echo "Missing $PROJECT_DIR/.env" >&2; exit 1; }
[[ -f "$ROS_SETUP" && -f "$WORKSPACE_SETUP" ]] || {
  echo "Build first: colcon build --symlink-install" >&2; exit 1;
}

wrap_command() {
  local title="$1" command="$2" pane_script
  printf -v pane_script 'cd %q; source %q; source %q; export ROS_DOMAIN_ID=%q; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; printf "\\033]0;%s\\007" %q; %s; status=$?; printf "\\n===== %s exited (%s); shell kept open =====\\n" "$status"; exec %q -l' \
    "$PROJECT_DIR" "$ROS_SETUP" "$WORKSPACE_SETUP" "${ROS_DOMAIN_ID:-20}" \
    "$title" "$title" "$command" "$title" "${SHELL:-/bin/bash}"
  printf 'bash -lc %q' "$pane_script"
}

# This pane starts vision only after its RealSense PointCloud2 topic exists.
REALSENSE_CMD='ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/ -r __node:=camera -p enable_color:=true -p enable_depth:=true -p depth_module.depth_profile:=848x480x15 -p rgb_camera.color_profile:=1280x720x15 -p align_depth.enable:=true -p enable_rgbd:=true -p enable_sync:=true -p pointcloud.enable:=true -p pointcloud.stream_filter:=2 -p enable_accel:=false -p enable_gyro:=false -p initial_reset:=true & camera_pid=$!; trap "kill $camera_pid 2>/dev/null || true" EXIT; until ros2 topic list | grep -qx /camera/depth/color/points; do echo "Waiting for RealSense point cloud..."; sleep 1; done; docker compose up -d vision; wait "$camera_pid"'

TITLES=("MoveIt2 / M0609" "RealSense + vision" "DB node" "Position estimation" "Voice command" "Echo command result" "Echo component result" "Echo task status" "Controller")
CMDS=(
  'ros2 launch dsr_moveit_config_m0609 start.launch.py mode:=real name:=dsr01 model:=m0609 gripper:=rg2 host:=192.168.1.100 gui:=true'
  "$REALSENSE_CMD"
  'set -a; source .env; set +a; ros2 run kit_db db_node'
  'ros2 run kit_robot position_estimation'
  'ros2 run kit_voice get_command'
  'ros2 topic echo /kit/command_result'
  'ros2 topic echo /kit/component_result'
  'ros2 topic echo /kit/task_status'
  'ros2 run kit_robot controller --ros-args --params-file src/kit_robot/resource/controller.yaml -p restart_delay_sec:=60.0'
)

LAYOUT_CONF="$(mktemp /tmp/e2e-terminator-XXXXXX.conf)"
terminal_entry() {
  local key="$1" parent="$2" order="$3" title="$4" command="$5"
  printf '    [[[%s]]]\n      type = Terminal\n      parent = %s\n      profile = default\n      uuid = %s\n      order = %s\n      command = """%s"""\n' \
    "$key" "$parent" "$(uuidgen)" "$order" "$(wrap_command "$title" "$command")"
}

{
  cat <<'EOF'
[layouts]
  [[e2e]]
    [[[window0]]]
      type = Window
      parent = ""
      order = 0
      maximised = True
    [[[top_bottom]]]
      type = VPaned
      parent = window0
      order = 0
      position = 300
    [[[top_row]]]
      type = HPaned
      parent = top_bottom
      order = 0
      position = 533
    [[[top_right]]]
      type = HPaned
      parent = top_row
      order = 1
      position = 533
    [[[middle_row]]]
      type = VPaned
      parent = top_bottom
      order = 1
      position = 300
    [[[middle_content]]]
      type = HPaned
      parent = middle_row
      order = 0
      position = 533
    [[[middle_right]]]
      type = HPaned
      parent = middle_content
      order = 1
      position = 533
    [[[bottom_row]]]
      type = HPaned
      parent = middle_row
      order = 1
      position = 533
    [[[bottom_right]]]
      type = HPaned
      parent = bottom_row
      order = 1
      position = 533
EOF
  terminal_entry terminal1 top_row 0 "${TITLES[0]}" "${CMDS[0]}"
  terminal_entry terminal2 top_right 0 "${TITLES[1]}" "${CMDS[1]}"
  terminal_entry terminal3 top_right 1 "${TITLES[2]}" "${CMDS[2]}"
  terminal_entry terminal4 middle_content 0 "${TITLES[3]}" "${CMDS[3]}"
  terminal_entry terminal5 middle_right 0 "${TITLES[4]}" "${CMDS[4]}"
  terminal_entry terminal6 middle_right 1 "${TITLES[5]}" "${CMDS[5]}"
  terminal_entry terminal7 bottom_row 0 "${TITLES[6]}" "${CMDS[6]}"
  terminal_entry terminal8 bottom_right 0 "${TITLES[7]}" "${CMDS[7]}"
  terminal_entry terminal9 bottom_right 1 "${TITLES[8]}" "${CMDS[8]}"
} > "$LAYOUT_CONF"

cd "$PROJECT_DIR"
docker compose up -d postgres mongodb
docker compose ps postgres mongodb
terminator -g "$LAYOUT_CONF" -l e2e &
echo "Started PostgreSQL/MongoDB and opened 9 Terminator panes."
