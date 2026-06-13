# car 开发文档索引

本目录记录 car(地面跟随/装货车)工程的设计决策与开发记录。新增文档请同步更新本索引。

> 与 `fly_car/docs/README.md` 同一套约定。AI/新人接手先读本文件,再读具体文档。

---

## 给 AI 的上下文(冷启动必读)

**这是什么项目**:`car/` 是比赛中与 `fly_car/`(陆空两用飞车)配套的**地面车**,ROS 2 Humble,独立开发板。职责:① 飞车地面行驶阶段跟随飞车;② 飞车起飞前对接装货(后续)。两车各带一个 bluesea 单线雷达(安装高度不同,扫描平面互不干扰),各自独立跑 Cartographer。

**硬约束(必须遵守)**:
- **双设备开发**:本地只写代码 + git;编译运行在开发板(`colcon build`),代码用 syncpi/rsync 传。**本地不要执行 `colcon build`,也不要声称"已编译验证"**。
- **双机隔离**:car 与 fly_car 的开发板(OrangePi)使用**不同的 `ROS_DOMAIN_ID`**(车板现用 10,飞车侧取非 10 值),绝不让两机的 `/tf`、`/scan` 进同一个 DDS 域。跨机只传飞车位姿(UDP 桥或 domain_bridge,待定),车端以 `/leader_pose` 为稳定接口。
- **底盘**:SR5E1E3 差速底盘(**不能横移**),文本协议见[调试指南](../src/orangepi_to_car/SR5E1E3_CHASSIS_DEBUG_GUIDE.md);控制器要按差速(v/w)设计,不要照搬 fly_car 的全向 PID。
- **坐标系/单位**:与 fly_car 相同——map 系算误差,串口桥下发前做 map→body 旋转;对外话题 cm/deg,TF 为 m/rad。

## 文档分类

### 架构 / 任务设计
- [车端跟随飞车功能设计](follow_fly_car_design.md) — 跟随功能总设计:双机位姿桥、map 系对齐、面包屑轨迹跟随、工程骨架与分阶段计划。**当前主线,先读这篇。**

### 功能包 / 硬件资料
- [SR5E1E3 底盘调试指南](../src/orangepi_to_car/SR5E1E3_CHASSIS_DEBUG_GUIDE.md) — 底盘串口协议全集($VW/$STOP/$MODE/IMU/超时保护)、PID 调参、接线。底盘是**差速驱动,不能横移**。
- [orangepi_to_car 包说明](../src/orangepi_to_car/README.md) — 底盘桥现有 /car_movement 离散命令接口。
- `wifi_ap_manager`(暂无独立文档):用 nmcli 把 `wlan0` 开成开放热点(AP),SSID `OPi_ROS2_TEST`,固定 `192.168.50.1/24`,`ipv4.method=shared` 自带 DHCP/NAT,作整个局域网的基站+网关。节点 `ap_manager`,action `start`/`stop`/`status`,`ros2 launch wifi_ap_manager open_ap.launch.py` 拉起。对端飞车连接见 `fly_car/src/wifi_sta_manager`。**开机自启动 + 心跳**见 `scripts/`：`wifi-ap.service`(systemd,Restart=always) 常驻 `wifi_keepalive.sh`,AP 掉了自动重拉(ping 飞车 `.2` 仅作日志,飞车没开不重建 AP);`start_wifi.sh`/`stop_wifi.sh` 手动控制。`wifi_keepalive.sh` 的 `WS_SETUP` 与 unit 的 `ExecStart` 路径按车板工作区改。

### 修改 / 开发记录
- [车上原有代码融合记录](onboard_code_merge_record.md) — flycar_car(1).zip 的取舍清单与由此引发的设计更新(差速底盘、域 ID=10 等)。

## 当前进度

| 模块 | 状态 |
| --- | --- |
| 跟随功能设计文档 | ✅ 方案已确认(2026-06-11;传输层 UDP/domain_bridge 待 M1 实测定夺) |
| M0 工程骨架 + 建图定位 | 🔶 本地完成(2026-06-11),待上板 `colcon build` + 实测建图。已建包:car_carto_pkg / follower_pkg(两节点骨架) / car_launch;bluesea2 自 fly_car 复制;orangepi_to_car 自车上代码融合(见[融合记录](onboard_code_merge_record.md)) |
| M1 位姿桥(fly_car 侧 pose_sender_pkg + 车侧 leader_pose_receiver,UDP) | 🔶 代码完成(2026-06-11),待组网+双板联调;发送端 target_ip 按组网实配 |
| M2 follower_node 面包屑跟随 + 安全(超时刹停/过近保持) | 🔶 代码完成(2026-06-11),待上板 RViz 验证 |
| M3 diff_drive_controller(/target_position→/cmd_vel)+ 底盘桥 $VW 流式通道 | 🔶 代码完成(2026-06-11),待实车闭环调参;chassis_timeout 仅 follow.launch 开启 |
| 联调前置:组网(热点/静态IP/双板域ID)、align_dx/dy/dyaw 摆位标定 | 🔶 组网已落地:车端 `wifi_ap_manager`(AP,192.168.50.1) + 飞车 `wifi_sta_manager`(STA,静态 192.168.50.2),pose_sender target_ip 默认改 192.168.50.1。两边 launch 拉起,待上板实测;摆位标定仍需硬件 |
| 跟随触发逻辑(谁发 /follow_enable,车端任务状态机) | ⬜ 待实现,依赖比赛流程定义;当前手动 `ros2 topic pub` |
| 双向任务通信(起飞/装货完成等握手,UDP 包型扩展,reserved 字段预留) | ⬜ 待实现,M4 前置 |
| M4 紧贴对接装货(DOCK/LOAD:近距传感器选型、对接控制、$SERVO 装货动作、撤离) | ⬜ 后续,另立文档 |
| Phase 2 增强(车雷达直接检测飞车修正对齐;leader 附近扇区伪墙滤波) | ⬜ 可选,联调发现问题再做 |
