# 车端跟随飞车功能设计(follow_fly_car)

> 状态:方案已于 2026-06-11 与用户讨论确认(坐标对齐=固定摆位静态变换;跟随=面包屑轨迹;底盘协议推迟到 M3;**双机传输层 UDP/domain_bridge 二选一待定**,以 `/leader_pose` 为稳定接口隔离该决策)。
> 本文是 `car/` 工程的第一篇设计文档,同时定义了 car 工程的整体骨架。

---

## 一、背景与目标

比赛中有两台设备:

- **fly_car(飞车)**:陆空两用,已有完整工程(`fly_car/`),Cartographer 定位、PID 位置控制、串口下发 STM32 飞控。
- **car(本工程,车)**:纯地面车,职责是
  1. **跟随**:飞车在地面行驶阶段,车保持一定距离跟在飞车后面;
  2. **装货**(后续):飞车起飞前,车精确对接到装货位,完成装货后撤离。

本期只做 **跟随功能**,但工程骨架按完整任务规划,装货作为状态机的预留状态。

两车各自带一个 bluesea 单线雷达,**安装高度不同,扫描平面不互扰**,各自独立跑 Cartographer 建图定位。

---

## 二、三个核心问题与选型

跟随的本质:车要实时知道"飞车在**车自己的 map 系**下的位姿"。这拆成三个问题:

### 2.1 双机通信:飞车位姿怎么传到车上

**组网前提(两个方案共同需要)**:两块 OrangePi 同网段。计划加 WiFi 模块由一台开热点、另一台接入(或共同接路由器),双方配静态 IP。组网是 M1 的前置工作,与下面选哪个传输方案无关。

**硬约束**:无论选哪个方案,**两台设备必须用不同的 `ROS_DOMAIN_ID`**——否则双方的 `/scan`、`/tf`、`map`/`odom`/`laser_link`/`base_link` 全部同名,TF 树直接互相污染,Cartographer 必炸。现状:车上原有 `flycar_system.launch.py` 已设 `ROS_DOMAIN_ID=10`,沿用之;**飞车侧须确认/设为非 10 的值**。

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| A. 自写 UDP 位姿桥 | 飞车侧加一个小节点定频(20 Hz)查 `map←laser_link` TF,打成 UDP 包发给车;车侧节点收包、转换坐标、发布 `/leader_pose` | 实现 ~100 行;只传一条 24 字节的数据,不依赖 DDS 跨 WiFi 发现;丢包无所谓,下一包就是最新状态 |
| B. domain_bridge 桥接指定话题 | 官方包,YAML 配置桥接飞车域的位姿话题到车的域(飞车侧需新增一个发布 pose 话题的小节点,查 TF 转话题) | 不用写网络代码;但两块板要装包配环境,底层仍是 DDS over WiFi,发现/QoS 在现场 WiFi 上的稳定性需实测 |
| ~~C. 同域 + 命名空间 + frame 前缀~~ | 全部话题和 frame 加 `fly_`/`car_` 前缀 | 已排除:Cartographer/bluesea2/现有 fly_car 代码全要改 frame 名,侵入大;全量 /scan /tf 流量走 WiFi |

**决策状态:代码已按方案 A(UDP)实现**(2026-06-11,发送端 `fly_car/src/pose_sender_pkg`,接收端 `follower_pkg/leader_pose_receiver`);**`/leader_pose`(车侧,`geometry_msgs/PoseStamped`)是稳定接口**,车端所有跟随逻辑只订阅这个话题——若实测想换 domain_bridge,只需替换 `leader_pose_receiver` 的 UDP 轮询为话题订阅,下游不动。

UDP 包格式(小端,定长 24 字节,收发两侧 `PosePacket` 结构体必须同步修改):

```
[magic u16 = 0xFC01][seq u16][stamp_ms u32][x_m f32][y_m f32][yaw_rad f32][reserved f32]
```

- `seq` 用于丢弃乱序旧包;`stamp_ms` 为发送方单调时钟,车侧只用"收包本地时间"判超时,**不做跨机时钟同步**(跟随不需要)。
- 两个方案下飞车侧都是纯增量(新增一个查 TF→发包/发话题的小节点),不动现有链路。

### 2.2 坐标对齐:飞车的 map 系 ≠ 车的 map 系

各自 Cartographer 的 map 原点 = 各自上电时刻的位姿,两个 map 系天然不重合。

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| **A. 已知初始摆位 + 静态变换(推荐基线)** | 比赛出发位置固定:两车按场地标线摆放,提前量好初始相对位姿,得到静态变换 `T_carmap←flymap`(参数 `align_dx / align_dy / align_dyaw`,可调) | 零开发量。误差 = 摆放误差 + 两图各自漂移,对 0.5~1 m 的跟随距离,10 cm 级误差完全可容忍 |
| B. 共享 pbstream,双机纯定位模式 | 提前建好场地图,两车都加载同一 `.pbstream` 定位,map 系天然重合 | 理论最优,但**两车雷达高度不同,扫到的墙体/障碍截面不同**,用对方高度建的图定位质量存疑,需要实测验证;且依赖赛前能建图 |
| C. 车端雷达直接检测飞车(增强项) | 车的扫描平面若能扫到飞车机体,以共享位姿为先验在其附近搜索运动聚类,直接得到飞车在**车自己 map 系**的位置,彻底消除两图错位 | 最准,但要先确认车雷达高度能打到飞车机体;作为 **Phase 2** 增强,不阻塞主线 |

**选 A 起步**,接收节点里把对齐做成 3 个参数,后续换 B/C 只动这一个节点。

### 2.3 跟随策略:跟"点"还是跟"轨迹"

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| **A. 面包屑轨迹跟随(推荐)** | 车记录飞车的历史轨迹点(每移动 `breadcrumb_spacing`≈10 cm 记一个),沿着这串点追,始终与飞车保持弧长 `d_follow` 的距离 | 车走的是飞车**走过的路**:飞车能过的地方车一定能过,不会内切弯、不会抄直线撞上飞车绕开过的障碍;飞车原地转身时跟随点不动,车不乱跑 |
| B. 固定偏移点跟随 | 目标 = 飞车当前位姿后方 `d_follow` 处 | 实现最简单,但转弯内切;飞车原地转 180° 时目标点瞬间甩到另一侧,车画大弧 |

**选 A**。跟随节点定频发布 `/target_position`(`[x_cm, y_cm, 0, yaw_deg]`,与 fly_car 约定一致的 cm/deg 单位)作为"移动靶"。

> **2026-06-11 更新(融合车上底盘代码后)**:车底盘 SR5E1E3 是**差速驱动**(`$VW,v,w`,左右轮+轴距),**不能横移**。澄清:fly_car 工程里的 `pid_control_pkg → uart_to_stm32` 是**飞控链路**(输出 map 系 vx/vy 给 STM32 飞控),与地面车控制本来就是两回事,不存在"移植过来用"——car 的控制器是全新的。M3 写**差速跟踪控制器**:距离误差→v、朝向误差→w(carrot-chasing),输出经 `orangepi_to_car` 串口桥下发 `$VW`。从 fly_car 沿用的只有**约定**:`/target_position` 的 `[x_cm,y_cm,z,yaw_deg]` 单位格式、map 系算误差、TF 取位姿的方式。面包屑"目标 yaw=指向 leader"的设定恰好契合差速车"先转向再前进"的控制方式。

---

## 三、car 工程结构规划

镜像 fly_car 的包布局,能移植的直接移植:

```
car/                          # ROS 2 Humble 工作区(开发板编译)
├── docs/                     # 本目录
└── src/
    ├── bluesea2/             # 雷达驱动,从 fly_car 复制(串口号/参数按车的雷达改)
    ├── car_carto_pkg/        # Cartographer 配置:lua + urdf + launch
    │                         #   基于 my_carto_pkg 改:tracking_frame 仍叫 laser_link
    │                         #   (两板不同域,frame 同名无冲突)
    ├── follower_pkg/         # ★ 本期核心,三个节点(详见 §四):
    │                         #   leader_pose_receiver  UDP→/leader_pose(M1)
    │                         #   follower_node         面包屑→/target_position(M2)
    │                         #   diff_drive_controller /target_position→/cmd_vel(M3)
    ├── orangepi_to_car/      # 底盘串口桥(2026-06-11 自车上原有代码融合):
    │                         #   SR5E1E3 文本协议,Python/termios,/dev/ttyS6 115200
    │                         #   ROS 接口:/car_movement(String 离散命令 0~5,原有)
    │                         #           + /cmd_vel(Twist 连续 $VW 流,M3 新增)
    │                         #   底盘协议指南:包内 SR5E1E3_CHASSIS_DEBUG_GUIDE.md
    │                         #   (fly_car 的 pid/uart_to_stm32 是飞控链路,与地面车无关,
    │                         #    不移植;serial_comm 也不需要,已从 car/src 移除)
    └── car_launch/           # 总启动入口 follow.launch.py(carto+跟随三节点+底盘桥)
```

fly_car 侧的增量(已实现,单独提交,不混入 car 的改动):

```
fly_car/src/pose_sender_pkg/ — 节点 pose_sender:
  定频(20Hz)查 map←laser_link TF,按 §2.1 包格式 UDP 单播到车的 IP:port
  (target_ip 参数必须按组网实配,launch 默认 192.168.4.2 是占位)
```

---

## 四、follower_pkg 节点设计

两个节点,职责分离:**收位姿的不管控制,做控制的不管网络**。

### 4.1 `leader_pose_receiver`(传输层适配节点,§2.1 决策只影响这一个节点)

| 项 | 内容 |
| --- | --- |
| 输入 | UDP 方案:监听 `udp_port` 收包;domain_bridge 方案:订阅桥过来的 pose 话题 |
| 处理 | (UDP:校验 magic → 按 `seq` 丢弃乱序旧包)→ 用 `T_carmap←flymap` 把位姿转到车 map 系 |
| 输出 | `/leader_pose`(`geometry_msgs/PoseStamped`,frame_id=`map`,stamp=收包本地时间)——**稳定接口,下游不感知传输方案** |
| 参数 | `udp_port`(默认 8888)、`align_dx`、`align_dy`、`align_dyaw` |

### 4.2 `follower_node`

| 项 | 内容 |
| --- | --- |
| 输入 | `/leader_pose`;自身位姿走 TF `map←laser_link`(与 fly_car 的 PID 同款取法);`/follow_enable`(`std_msgs/Bool`,使能开关,预留给上层任务逻辑) |
| 输出 | `/target_position`(`Float32MultiArray [x_cm, y_cm, 0, yaw_deg]`,定频 20 Hz) |

**面包屑逻辑**:

1. 每收到 `/leader_pose`,若与队尾点距离 ≥ `breadcrumb_spacing`,入队(队列存 map 系坐标,限长)。
2. 跟随目标点 = 沿队列从 leader 当前位置向回走弧长 `d_follow` 处的点;队列总弧长 < `d_follow` 时(刚启动或飞车没怎么动)→ 保持不动。
3. 目标 yaw = 指向 leader 当前位置(车头始终朝着飞车,为后续对接装货留好姿态)。
4. 车追上目标点后,目标点随飞车前进自然前移——`/target_position` 是一个"移动的胡萝卜"。

**状态机**(本期只实现前两个,后两个预留):

```
IDLE ──/follow_enable=true──► FOLLOW ──(预留)──► DOCK ──► LOAD/RETREAT
  ▲                              │
  └──超时/disable/过近──────────┘ (安全停车后回 IDLE 或原地保持)
```

**安全策略**(全部在 follower_node 内,不依赖上层):

| 条件 | 动作 |
| --- | --- |
| `/leader_pose` 超时 > `leader_timeout`(0.5 s) | 目标点 = 当前自身位姿(原地刹停) |
| 与 leader 直线距离 < `d_min`(防追尾) | 同上,原地保持,等飞车走远再恢复 |
| 未使能 / 无自身 TF | 不发布目标(PID 端无目标即不动) |

### 4.3 `diff_drive_controller`(M3,差速跟踪控制器)

| 项 | 内容 |
| --- | --- |
| 输入 | `/target_position`(移动靶,cm/deg);自身位姿走 TF `map←laser_link` |
| 输出 | `/cmd_vel`(`geometry_msgs/Twist`,`linear.x`=v m/s,`angular.z`=w rad/s,20 Hz) |

控制律(carrot-chasing,贴合差速底盘"先转向再前进"):

- 距目标点 `d > pos_tol`:方位误差 `e_h = normalize(atan2(dy,dx) − yaw)`;`w = clamp(kp_w·e_h)`;`|e_h| > align_gate` 时 `v = 0`(原地转向),否则 `v = clamp(kp_v·d)·cos(e_h)`(对准程度越好走得越快)。
- `d ≤ pos_tol`:原地对准目标 yaw,误差小于 `yaw_tol` 后 v=w=0。
- 安全:`/target_position` 超时(`target_timeout`)→ 连续发 1 s 零速刹停,然后**停止发布**(释放 `/cmd_vel`,避免与 `/car_movement` 离散命令长期抢底盘);有目标但 TF 失效 → 发零速。

### 4.4 `orangepi_to_car` 桥的 `/cmd_vel` 通道(M3 扩展)

- 收到 Twist:限幅(v∈[−2,2] m/s,w∈[−3,3] rad/s)→ 首次/失效后自动 `$MODE,VW` → `$VW,v,w`。
- **快速通道**:`$VW` 流式发送不逐帧等应答(原 `send_command` 每帧等 0.35 s,20 Hz 跟不上),只顺手清接收缓冲;发送限频 40 ms,零速帧不受限频(保证刹停到位)。
- **桥侧看门狗**:`cmd_vel` 断流 > 0.5 s → 发 `$STOP`。与控制器的零速 burst、底盘侧 `$SET,TIMEOUT` 构成三层防线。
- `--chassis-timeout-ms`(默认 0 关):>0 时启动后下发 `$SET,TIMEOUT,1,ms`。**默认关是为了不破坏原有离散命令用法**($LINE 后静默会被底盘自动刹停);跟随任务的 `follow.launch.py` 以 500 ms 开启。
- 离散 `/car_movement` 与 `/cmd_vel` 并存:离散命令到达时重置流式状态(下次 cmd_vel 重新进 VW 模式),后到者生效,不做互斥仲裁。

### 4.5 参数表(待实测标定的集中在这)

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `d_follow_cm` | 80 cm | 跟随弧长距离 |
| `d_min_cm` | 40 cm | 最小安全直线距离 |
| `breadcrumb_spacing_cm` | 10 cm | 面包屑间距 |
| `prune_margin_cm` | 50 cm | 面包屑修剪余量(弧长超 d_follow+margin 的旧点丢弃) |
| `leader_timeout_s` | 0.5 s | 位姿超时判定 |
| `publish_rate_hz` | 20 Hz | 目标点/速度发布频率 |
| `align_dx/dy/dyaw` | 0/0/0 | 两 map 系静态对齐(按比赛摆位量取) |
| `udp_port` | 8888 | 位姿桥端口(发送端 `target_ip` 按组网实配) |
| `kp_v` / `v_max_mps` | 1.0 / 0.4 | 距离→线速度增益与限幅 |
| `kp_w` / `w_max_rps` | 1.5 / 1.0 | 角度→角速度增益与限幅 |
| `align_gate_deg` | 45° | 方位误差门限,超过先原地转向 |
| `pos_tol_cm` / `yaw_tol_deg` | 5 cm / 8° | 到位判定 |
| `target_timeout_s` / `stop_burst_s` | 1.0 / 1.0 s | 控制器目标超时与零速 burst 时长 |

---

## 五、风险与观察项

1. **车雷达会持续扫到飞车机体**(雷达平面不互扰 ≠ 机体不被扫到):跟随时正前方 `d_follow` 处始终有一团动态点,可能在车的地图上留"伪墙"、干扰 scan match。观察项:实测建图质量;若有影响,在 bluesea2 或 carto 前加一个按 `/leader_pose` 附近扇区滤波的中间节点(预留方案,先不做)。
2. ~~车底盘串口协议未定~~ **已解决(2026-06-11)**:底盘是 SR5E1E3 驱动板,文本协议(`$CMD,args\r\n`),支持连续 `$VW,v,w`(m/s、rad/s)、`$STOP`、状态/编码器查询、板载 IMU 航向(`$GET,IMU`→`$ATT,yaw=...`)。两个落地注意:① **差速底盘不能横移**,M3 控制器按差速设计(见 §2.3 更新);② 底盘自带 `$SET,TIMEOUT` 通信超时自动停车,实车联调务必开启,作为 ROS 侧安全策略之外的最后防线。板载 IMU 的 yaw 还可作为 Cartographer 之外的航向冗余,备用。
3. **紧贴对接(DOCK)不能只靠 map 系位姿**:固定摆位对齐 + 两图漂移是 10 cm 级误差,且单线雷达近距有盲区,做不到"紧贴"。已确认 DOCK 阶段需要**额外的近距传感器**(如激光测距/ToF 模块,装在车上对飞车测距),具体选型与安装位置到 M4 再定。跟随阶段(本期)不受影响。
4. 两 map 漂移累积:长时间跑后对齐误差变大;Phase 2 的雷达直接检测(§2.2 方案 C)是根治手段。
5. 飞车倒车/急转时面包屑队列的退化情况,联调时重点测。

---

## 六、分阶段实施计划

| 阶段 | 内容 | 验证方式(均在开发板) |
| --- | --- | --- |
| M0 | 搭 car 工作区骨架;复制 bluesea2、serial_comm;car_carto_pkg 配好 | 车上 `colcon build` 通过;RViz 看车自己建图定位正常 |
| M1 | **组网**(WiFi 模块热点/路由器 + 静态 IP + 双板不同 DOMAIN_ID)→ 飞车侧 `pose_sender` + 车侧 `leader_pose_receiver`;UDP / domain_bridge 在此阶段实测定夺 | 两板各自跑起来,车上 `ros2 topic echo /leader_pose`,人推飞车,数值连续且方向正确 |
| M2 | `follower_node`(面包屑 + 状态机 + 安全) | RViz 同时显示 `/leader_pose` 与 `/target_position`,人推飞车走折线,目标点沿轨迹滞后跟随、无跳变 |
| M3 | 新写差速跟踪控制器(`/target_position`+自身 TF → v/w);`orangepi_to_car` 增加连续速度话题接口(→`$VW`);开启底盘 `$SET,TIMEOUT`,实车闭环 | 实车跟随,调 `d_follow`/控制参数 |
| M4(后续) | DOCK 紧贴对接装货(需近距传感器,如激光测距/ToF,见 §五-3) | 另立文档 |

---

## 七、硬约束提醒(与 fly_car 一致)

- **双设备开发**:本地只写代码 + git;开发板 `colcon build` 与实测,syncpi/rsync 传代码。本地不编译、不声称"已编译验证"。
- **坐标系**:位姿/误差在 map 系算,下发底盘前由串口桥做 map→body 旋转。
- **单位**:对外话题 cm、cm/s、deg;TF 为 m、rad,转换别漏。
