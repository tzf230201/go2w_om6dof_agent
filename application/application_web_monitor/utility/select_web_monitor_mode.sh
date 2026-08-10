#!/usr/bin/env bash
# Desktop launcher: choose monitor scope, persist it, restart the service, open it.
set -euo pipefail

choice="$(/usr/bin/zenity --list --radiolist \
  --title="Run Web Monitor" \
  --text="Choose which robot interface to display and control." \
  --column="Select" --column="Mode" --column="Description" \
  TRUE "OM6DOF only" "Arm, joystick, perception, and RealSense camera" \
  FALSE "Go2W only" "Go2W camera and controls" \
  FALSE "Go2W + OM6DOF" "Display both robot interfaces" \
  --width=640 --height=300)" || exit 0

case "$choice" in
  "OM6DOF only") mode="om6dof" ;;
  "Go2W only") mode="go2w" ;;
  "Go2W + OM6DOF") mode="both" ;;
  *)
    /usr/bin/zenity --error --text="The selected web monitor mode is not recognized."
    exit 1
    ;;
esac

config_dir="$HOME/.config/om6dof-web-monitor"
mkdir -p "$config_dir"
umask 077
printf 'WEB_MONITOR_MODE=%s\n' "$mode" > "$config_dir/mode.env"

if ! /usr/bin/sudo -n /usr/bin/systemctl restart om6dof-web-monitor.service; then
  /usr/bin/zenity --error --title="Web Monitor" \
    --text="The web monitor service could not be started. Contact an administrator to grant permission to restart the service."
  exit 1
fi

sleep 2
/usr/bin/xdg-open http://127.0.0.1:8080/
