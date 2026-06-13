#!/usr/bin/env bash
# 车(AP)Wi-Fi 心跳守护:被 wifi-ap.service 以 root 常驻拉起。
# 职责:保证开放热点处于 up;AP 掉了就用 wifi_ap_manager 的 launch 重新拉起。
# 注意:ping 飞车 .2 仅用于状态日志,ping 不通 != AP 坏(飞车可能没开),不据此重建 AP。
set -u

# ===== 按开发板实际路径调整(车板工作区根) =====
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/orangepi/kian_flycar/install/setup.bash"   # TODO: 改成车板真实工作区
# ==============================================

CONN="OPi_ROS2_OPEN_AP"   # 与 open_ap.launch.py 的 connection_name 一致
PEER_IP="192.168.50.2"     # 飞车 STA 固定地址,仅作连通性日志
INTERVAL=5                  # 心跳周期(秒)

# shellcheck disable=SC1090
source "$ROS_SETUP"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

log() { echo "[wifi-ap $(date '+%F %T')] $*"; }

is_active() { nmcli -g NAME connection show --active 2>/dev/null | grep -qx "$CONN"; }
peer_ok()   { ping -c1 -W1 "$PEER_IP" >/dev/null 2>&1; }

start_ap() {
  log "(re)starting AP via launch ..."
  ros2 launch wifi_ap_manager open_ap.launch.py action:=start \
    >/tmp/wifi_ap_launch.log 2>&1 || log "launch returned non-zero (see /tmp/wifi_ap_launch.log)"
}

log "keepalive start: conn=$CONN peer=$PEER_IP interval=${INTERVAL}s"
while true; do
  if ! is_active; then
    log "AP '$CONN' not active -> start"
    start_ap
  elif peer_ok; then
    log "AP up, fly_car $PEER_IP reachable"
  else
    log "AP up, fly_car $PEER_IP not present (ok if fly_car is off)"
  fi
  sleep "$INTERVAL"
done
