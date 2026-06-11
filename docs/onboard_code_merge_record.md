# 车上原有代码融合记录(2026-06-11)

来源:`flycar_car(1).zip`(自车载 OrangePi 拷回,解压后 938M)。

## 保留(并入 `car/src/`)

| 内容 | 去向 | 说明 |
| --- | --- | --- |
| `orangepi_to_car.pkg/` 整包 | `src/orangepi_to_car/`(去掉目录名 `.pkg` 后缀,删 `__pycache__`) | SR5E1E3 底盘串口桥,ament_python 包。节点 `orangepi_to_carv2`:订阅 `/car_movement`(String 离散命令 0~5),termios 直驱 `/dev/ttyS6` 115200。含底盘协议指南 `SR5E1E3_CHASSIS_DEBUG_GUIDE.md` 与 README |

## 丢弃

| 内容 | 原因 |
| --- | --- |
| `.vscode/`(937M,几乎是整个 zip 的体积) | `browse.vc.db` 是 VS Code C++ 索引缓存 |
| `build/` `install/` `log/` `__pycache__/` | colcon 编译产物与日志 |
| `.git/`(仅 2 个 commit:Initial + VS Code 配置) | 无有价值历史,car 仓库已另行 git init |
| 根目录 `orangepi_to_car.py` | v1 旧版(274 行,被 v2 取代) |
| 根目录 `orangepi_to_car_test.py` | 与包内 `orangepi_to_carv2.py` 同源的旧副本(类名/文案差异) |
| 根目录 `SR5E1E3_CHASSIS_DEBUG_GUIDE(1).md` | 旧版指南(UART1);包内版本(UART6,Orange Pi 5 Max 引脚)更新,以包内为准 |
| `.agents/` `.codex/` | 其他 AI 工具的空配置目录 |

## 融合带来的设计更新(已同步进[设计文档](follow_fly_car_design.md))

1. **底盘协议已定**:SR5E1E3 文本协议,支持连续 `$VW,v,w` —— M3 不再阻塞。
2. **差速底盘,不能横移**:M3 控制器按差速(v/w)全新编写。(澄清:fly_car 的 `pid_control_pkg`/`uart_to_stm32` 是**飞控链路**,与地面车控制无关,本就不该移植;早前骨架里误复制的 `src/serial_comm/` 已移除。)
3. 车板已用 `ROS_DOMAIN_ID=10`(见 `flycar_system.launch.py`),双机隔离时飞车侧取非 10 值。
4. 底盘带 IMU 航向与 `$SET,TIMEOUT` 超时停车,联调时利用。

## 遗留注意

- `launch/flycar_system.launch.py` 里硬编码了板上路径的"语音终端"进程(`/home/orangepi/qianrushi/.../main.py`),与跟随无关,原样保留;跟随任务用 `car_launch/follow.launch.py`,不会拉起它。
- 节点启动时会**循环发 `$SERVO,1,160` 直到收到 `$OK` 才接受话题**——板子没接底盘时节点会卡在初始化,桌面调试注意。
