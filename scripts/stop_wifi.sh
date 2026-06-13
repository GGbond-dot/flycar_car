#!/usr/bin/env bash
# 停止车(AP)Wi-Fi:先杀 autostart_wifi 心跳(否则立刻又把 AP 拉起),再关掉热点。
set -o pipefail
CONN="OPi_ROS2_OPEN_AP"

# 按脚本名匹配(不依赖启动时是绝对/相对路径);本板仅一个 autostart_wifi.sh,安全
pkill -f autostart_wifi.sh && echo "已停心跳" || echo "心跳未在跑"
nmcli connection down "$CONN" 2>/dev/null && echo "已关闭 AP $CONN" || echo "$CONN 本就未开"
