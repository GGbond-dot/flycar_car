#!/usr/bin/env bash
# kian_flycar 车(AP)Wi-Fi 开机自启 + 心跳 — 仿 kian_26fly/scripts/autostart_fly.sh 套路。
#
# 加到桌面"会话与启动":命令直接填本脚本绝对路径即可(不需要 .desktop/.service)。
# 做的事:
#   1) source ROS humble + 本工作空间 install(开机自启是非交互 shell,必须显式 source)
#   2) 心跳循环:保证开放热点处于 up,AP 掉了就用 open_ap.launch.py 重新拉起
#      ping 飞车 .2 仅作连通性日志,ping 不通 != AP 坏(飞车可能没开),不据此重建 AP。
# 停止: 同目录 stop_wifi.sh。日志: ~/wifi_logs/wifi_ap.log。
#
# 注意:不要开 set -u(nounset)。ROS 的 setup.bash 会引用未定义变量,开了会直接报错退出。
set -o pipefail

# ---- 路径解析:脚本在 <ws>/scripts/ 下,ws 根 = 上一级(不写死路径)----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

LOG_DIR="${WIFI_LOG_DIR:-$HOME/wifi_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/wifi_ap.log"

CONN="OPi_ROS2_OPEN_AP"               # 与 open_ap.launch.py 的 connection_name 一致
PEER_IP="192.168.50.2"                 # 飞车 STA 固定地址(仅作连通性日志)
INTERVAL="${WIFI_HEARTBEAT_SEC:-5}"    # 心跳周期(秒)

source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"

log() { echo "[wifi-ap $(date '+%F %T')] $*" | tee -a "$LOG"; }

is_active() { nmcli -g NAME connection show --active 2>/dev/null | grep -qx "$CONN"; }
peer_ok()   { ping -c1 -W1 "$PEER_IP" >/dev/null 2>&1; }
start_ap()  { ros2 launch wifi_ap_manager open_ap.launch.py action:=start >>"$LOG" 2>&1; }

trap 'log "autostart_wifi 退出"; exit 0' INT TERM

log "=== autostart_wifi 启动: ws=$WS_ROOT conn=$CONN peer=$PEER_IP interval=${INTERVAL}s ==="
last=""
while true; do
  if ! is_active; then
    [ "$last" != "down" ] && log "AP '$CONN' 未激活 -> 重新拉起"
    last="down"; start_ap
  elif peer_ok; then
    [ "$last" != "ok" ] && log "AP up,飞车 $PEER_IP 已连通"
    last="ok"
  else
    [ "$last" != "lonely" ] && log "AP up,飞车 $PEER_IP 未上线(飞车没开属正常)"
    last="lonely"
  fi
  sleep "$INTERVAL"
done
