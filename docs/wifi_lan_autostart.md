# 双机自组局域网 + Wi-Fi 自启动（车侧）

车与飞车各自开机自组一个局域网做跨机通信（飞车位姿 UDP → 车跟随）。本文记录车端 AP 与自启动脚本的设计与坑。飞车端对应文档见 `../../fly_car/docs/wifi_lan_autostart.md`（含拓扑/地址/关键认知全表）。

## 一、拓扑与地址（简）

| 设备 | 角色 | IP |
| --- | --- | --- |
| 车板 | AP / 网关 / DHCP（`ipv4.method=shared`） | `192.168.50.1` |
| 飞车板 | STA 静态 | `192.168.50.2` |
| 监控 PC | STA（DHCP，`.10` 起） | `192.168.50.x` |

开放热点 SSID `OPi_ROS2_TEST`（无密码）。

## 二、`wifi_ap_manager` 功能包

用 nmcli 把 `wlan0` 开成开放热点，作整个局域网的基站+网关。

- 节点 `ap_manager`，action `start` / `stop` / `status`。
- 关键参数：`ssid`(`OPi_ROS2_TEST`)、`interface`(`wlan0`)、`connection_name`(`OPi_ROS2_OPEN_AP`)、`ip_cidr`(`192.168.50.1/24`)、`band`(`bg`=2.4G)、`channel`。
- 用法：`ros2 launch wifi_ap_manager open_ap.launch.py`。建 AP 时主动移除 `802-11-wireless-security` → 开放网络。`ipv4.method=shared` 自带 DHCP+NAT，飞车/PC 连入自动分址。

## 三、单网卡的两个硬约束（务必理解）

1. **AP/STA 互斥**：`wlan0` 同时只能发 AP 或连一个上网 wifi。**车板开 AP 期间没有外网**（正常，无解也不需要解；PC 连热点走 `192.168.50.1` rsync 传代码不依赖车板外网）。
2. **AP→STA 驱动切不干净**（orangepi5max 板载 wifi）：运行中把 AP 关掉再去连上网 wifi，会出 `Secrets were required` / `network could not be found` 等矛盾报错（即使密码已用 `psk-flags 0` 存成系统级、热点可见也连不上）。根因是驱动从 AP 模式退出后没干净恢复成 station。**故车端不做运行时切换，改用「标志位 + 重启」**：每次开机要么纯 AP、要么纯 STA，`wlan0` 从干净状态启动。

> 备查：上网 wifi 密码若是「钥匙串/agent-owned」模式，终端连会报 `Secrets were required`。转系统级：
> `nmcli connection modify "<WiFi名>" 802-11-wireless-security.psk "<密码>" 802-11-wireless-security.psk-flags 0 connection.autoconnect yes`
> （注意参数从左到右生效，psk 与 psk-flags 同条命令时若仍异常，删除连接重建：`nmcli device wifi connect "<名>" password "<密码>"`）

## 四、自启动脚本（`car/scripts/`）

仿 `kian_26fly/scripts/autostart_fly.sh` 套路的自包含单脚本：自动解析 `WS_ROOT`、显式 `source` ROS+install、不开 `set -u`、日志 `~/wifi_logs/wifi_ap.log`。**加到桌面「会话与启动」命令直接填脚本绝对路径**（无需 .desktop/.service，免 sudo）。

### 标志位机制（车端专属，规避 AP→STA 切换）

- 标志文件 `~/.flycar_wifi_autostart`（内容 `true`/`false`，放 home 不受 rsync 影响；缺省按 `true`）。
- `autostart_wifi.sh` 开机先读标志：
  - `true` → 开 AP + 心跳（每 5s 保证 AP 处于 up，掉了用 `open_ap.launch.py` 重拉；ping 飞车 `.2` 仅作日志，飞车没开不重建 AP）。
  - `false` → **直接退出，全程不碰 AP** → `wlan0` 是干净 STA，NetworkManager 自动连上网 wifi。
- `stop_wifi.sh true|false` 改标志（`false` 顺手停当前心跳/关 AP），**`sudo reboot` 才彻底生效**。

### 用法

```bash
# 要组网（默认）
./stop_wifi.sh true  && sudo reboot     # 或不重启，手动 ./autostart_wifi.sh
# 要上网开发
./stop_wifi.sh false && sudo reboot     # 重启后 wlan0 自动连上网 wifi
```

## 五、上板验证

1. 板上 `colcon build --packages-select wifi_ap_manager && source install/setup.bash`。
2. `./stop_wifi.sh true` → `./autostart_wifi.sh`（前台看日志）→ 飞车/手机能搜到 `OPi_ROS2_TEST`、`ip -br addr show wlan0` 为 `192.168.50.1`。
3. `./stop_wifi.sh false && sudo reboot` → 回来后 `wlan0` 拿到 IP、能上网，验证「关局域网→恢复上网」。
4. 排查看 `~/wifi_logs/wifi_ap.log`。

## 六、为什么车/飞车机制不对称

车要发 AP（有 AP→STA 驱动坑）→ 标志位 + 重启。飞车是纯 STA（STA↔STA 切换无坑）→ 实时自动择网（扫到车热点连局域网，否则连默认上网 wifi）。详见飞车端文档。
