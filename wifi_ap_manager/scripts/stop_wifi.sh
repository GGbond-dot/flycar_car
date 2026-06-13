#!/usr/bin/env bash
# 手动停止车(AP)Wi-Fi:先停心跳服务(否则它立刻又把 AP 拉起),再关掉热点。
set -u
SERVICE="wifi-ap.service"
CONN="OPi_ROS2_OPEN_AP"
sudo systemctl stop "$SERVICE"
sudo nmcli connection down "$CONN" 2>/dev/null \
  && echo "AP $CONN down" \
  || echo "$CONN already inactive"
echo "stopped $SERVICE"
