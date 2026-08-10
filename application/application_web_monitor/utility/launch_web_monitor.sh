#!/usr/bin/env bash
# Starts one web-monitor instance using the desktop-selected robot mode.
set -eo pipefail

config_file="$HOME/.config/om6dof-web-monitor/mode.env"
mode="om6dof"
if [[ -r "$config_file" ]]; then
  configured_mode="$(sed -n 's/^WEB_MONITOR_MODE=//p' "$config_file" | head -n 1)"
  [[ -n "$configured_mode" ]] && mode="$configured_mode"
fi

case "$mode" in
  go2w)
    go2w_enabled=true
    om6dof_enabled=false
    ;;
  both)
    go2w_enabled=true
    om6dof_enabled=true
    ;;
  om6dof)
    go2w_enabled=false
    om6dof_enabled=true
    ;;
  *)
    echo "Invalid WEB_MONITOR_MODE '$mode'; using om6dof." >&2
    go2w_enabled=false
    om6dof_enabled=true
    ;;
esac

source /opt/ros/humble/setup.bash
if [[ "$go2w_enabled" == "true" ]]; then
  source /home/kublab/unitree_ros2/cyclonedds_ws/install/setup.bash
fi
source /home/kublab/ros2_ws/install/setup.bash
# Let DDS select the active AGX interface when the Go2W Ethernet link is absent.
unset CYCLONEDDS_URI
exec ros2 run application_web_monitor web_monitor --ros-args \
  -p go2w_enabled:="$go2w_enabled" \
  -p om6dof_enabled:="$om6dof_enabled"
