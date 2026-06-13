#!/usr/bin/env bash
# 切换车(AP)Wi-Fi 自启动总开关(标志位),配合 autostart_wifi.sh 开机判断。
#   ./stop_wifi.sh false  -> 关:写标志 false + 停当前心跳/AP;重启后 wlan0 自动连上网 Wi-Fi
#   ./stop_wifi.sh true   -> 开:写标志 true;重启后(或手动跑 ./autostart_wifi.sh)自动开局域网
# 不带参数默认 false(关)。改完标志要 `sudo reboot` 才彻底生效(避开 AP<->STA 实时切换的驱动问题)。
set -o pipefail
FLAG_FILE="$HOME/.flycar_wifi_autostart"
CONN="OPi_ROS2_OPEN_AP"
ARG="${1:-false}"

case "$ARG" in
  true)
    echo true > "$FLAG_FILE"
    echo "已启用自启动局域网 (标志=true -> $FLAG_FILE)"
    echo "生效: sudo reboot  (或现在手动跑 ./autostart_wifi.sh)"
    ;;
  false)
    echo false > "$FLAG_FILE"
    echo "已禁用自启动局域网 (标志=false -> $FLAG_FILE)"
    pkill -f autostart_wifi.sh && echo "已停当前心跳" || echo "心跳未在跑"
    nmcli connection down "$CONN" 2>/dev/null && echo "已关闭 AP $CONN" || echo "$CONN 本就未开"
    echo "重启后 wlan0 将自动连上网 Wi-Fi: sudo reboot"
    ;;
  *)
    echo "用法: $0 [true|false]   (true=开机开局域网, false=开机不开/留给上网)"
    exit 1
    ;;
esac
