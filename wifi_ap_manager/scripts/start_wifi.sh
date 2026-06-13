#!/usr/bin/env bash
# 手动启动车(AP)Wi-Fi 心跳服务(开机已 enable 时无需手动跑)。
set -e
SERVICE="wifi-ap.service"
sudo systemctl start "$SERVICE"
sudo systemctl --no-pager --lines=10 status "$SERVICE" || true
echo "started $SERVICE (live log: journalctl -u $SERVICE -f)"
