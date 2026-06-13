#!/usr/bin/env bash
# 停止车(AP)Wi-Fi:先杀 autostart_wifi 心跳(否则立刻又把 AP 拉起),再关掉热点。
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONN="OPi_ROS2_OPEN_AP"

pkill -f "$SCRIPT_DIR/autostart_wifi.sh" && echo "已停心跳" || echo "心跳未在跑"
nmcli connection down "$CONN" 2>/dev/null && echo "已关闭 AP $CONN" || echo "$CONN 本就未开"
