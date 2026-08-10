#!/usr/bin/env bash
# Desktop launcher for MoveIt with an OM6DOF safety/status check.
set -eo pipefail

repo_root="/home/kublab/ros2_ws/src/go2w_om6dof_agent/application/application_web_monitor"
rest_checker="$repo_root/utility/moveit_rest_check.py"
hardware_service="om6dof-hardware.service"

source /opt/ros/humble/setup.bash
source /home/kublab/ros2_ws/install/setup.bash
unset CYCLONEDDS_URI

stack_state="$(/usr/bin/systemctl is-active "$hardware_service" 2>/dev/null || true)"
[[ -n "$stack_state" ]] || stack_state="inactive"

torque_state="UNKNOWN: no Dynamixel feedback"
rest_state="UNKNOWN: stack is not active"
if [[ "$stack_state" == "active" ]]; then
  dxl_feedback="$(timeout 4 ros2 topic echo --once /dynamixel_hardware_interface/dxl_state 2>/dev/null || true)"
  if grep -q '^torque_state:' <<<"$dxl_feedback"; then
    torque_values="$(sed -n '/^torque_state:/,/^[^ -]/p' <<<"$dxl_feedback" | grep -E '^-[[:space:]]+(true|false)' || true)"
    if [[ -n "$torque_values" ]] && ! grep -q 'true' <<<"$torque_values"; then
      torque_state="DISABLED: all reported Dynamixels"
    elif [[ -n "$torque_values" ]] && ! grep -q 'false' <<<"$torque_values"; then
      torque_state="ENABLED: all reported Dynamixels"
    elif [[ -n "$torque_values" ]]; then
      torque_state="MIXED: some Dynamixels enabled"
    fi
  fi

  joint_feedback="$(timeout 4 ros2 topic echo --once /joint_states 2>/dev/null || true)"
  rest_state="$(printf '%s' "$joint_feedback" | python3 "$rest_checker")"
fi

status_text="OM6DOF stack: $stack_state
Torque: $torque_state
MoveIt home/rest area: $rest_state

The rest-area check compares measured joint feedback with MoveIt's home_pose (±0.25 rad per joint)."

if [[ "${1:-}" == "--status" ]]; then
  printf '%b\n' "$status_text"
  exit 0
fi

current_default=FALSE
restart_default=FALSE
offline_default=FALSE
if [[ "$stack_state" == "active" ]]; then
  current_default=TRUE
else
  restart_default=TRUE
fi

choice="$(/usr/bin/zenity --list --radiolist \
  --title="Run OM6DOF MoveIt" \
  --text="$status_text" \
  --column="Select" --column="Action" --column="Description" \
  "$current_default" "Use the current OM6DOF stack" "Launch MoveIt without opening a second U2D2 connection" \
  "$restart_default" "Restart OM6DOF stack, then run MoveIt" "Use when hardware needs a clean restart" \
  "$offline_default" "Stop OM6DOF stack, then run planning-only MoveIt" "No hardware feedback or real execution" \
  --width=780 --height=390)" || exit 0

case "$choice" in
  "Use the current OM6DOF stack")
    if [[ "$stack_state" != "active" ]]; then
      /usr/bin/zenity --error --title="MoveIt" --text="OM6DOF stack is not active. Choose the restart option or planning-only mode."
      exit 1
    fi
    ;;
  "Restart OM6DOF stack, then run MoveIt")
    /usr/bin/zenity --question --title="Restart OM6DOF stack?" \
      --text="$status_text

Restarting interrupts arm control. Keep the workspace clear." || exit 0
    /usr/bin/sudo -n /usr/bin/systemctl --no-block restart "$hardware_service"
    sleep 6
    if [[ "$(/usr/bin/systemctl is-active "$hardware_service" 2>/dev/null || true)" != "active" ]]; then
      /usr/bin/zenity --error --title="MoveIt" --text="OM6DOF stack did not become active. MoveIt was not launched."
      exit 1
    fi
    ;;
  "Stop OM6DOF stack, then run planning-only MoveIt")
    /usr/bin/zenity --question --title="Stop OM6DOF stack?" \
      --text="$status_text

Stopping the stack disables control. If torque is enabled, support the arm because it may move under gravity." || exit 0
    /usr/bin/sudo -n /usr/bin/systemctl stop "$hardware_service"
    ;;
  *)
    exit 0
    ;;
esac

/usr/bin/gnome-terminal --title="OM6DOF MoveIt" -- bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/kublab/ros2_ws/install/setup.bash
  unset CYCLONEDDS_URI
  ros2 launch om6dof_bringup real.launch.py start_hardware:=false start_rviz:=true use_sim:=false
  exec bash
'
