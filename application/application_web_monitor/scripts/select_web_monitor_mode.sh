#!/usr/bin/env bash
# Desktop launcher: choose monitor scope, persist it, restart the service, open it.
set -euo pipefail

choice="$(/usr/bin/zenity --list --radiolist \
  --title="Run Web Monitor" \
  --text="Pilih robot yang ingin ditampilkan dan dikontrol." \
  --column="Pilih" --column="Mode" --column="Keterangan" \
  TRUE "OM6DOF saja" "Arm, joystick, perception, dan kamera RealSense" \
  FALSE "Go2W saja" "Kamera dan kontrol Go2W" \
  FALSE "Go2W + OM6DOF" "Tampilkan keduanya" \
  --width=640 --height=300)" || exit 0

case "$choice" in
  "OM6DOF saja") mode="om6dof" ;;
  "Go2W saja") mode="go2w" ;;
  "Go2W + OM6DOF") mode="both" ;;
  *)
    /usr/bin/zenity --error --text="Pilihan mode web monitor tidak dikenali."
    exit 1
    ;;
esac

config_dir="$HOME/.config/om6dof-web-monitor"
mkdir -p "$config_dir"
umask 077
printf 'WEB_MONITOR_MODE=%s\n' "$mode" > "$config_dir/mode.env"

if ! /usr/bin/sudo -n /usr/bin/systemctl restart om6dof-web-monitor.service; then
  /usr/bin/zenity --error --title="Web Monitor" \
    --text="Service web monitor tidak dapat dijalankan. Hubungi administrator untuk izin restart service."
  exit 1
fi

sleep 2
/usr/bin/xdg-open http://127.0.0.1:8080/
