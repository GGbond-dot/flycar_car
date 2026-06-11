# SR5E1E3 小车底盘调试指南

## 1. 整体调试流程

### 1.1 上电前检查

1. 确认电机驱动、电机、电源、编码器接线正确。
2. 确认小车轮子悬空，避免电机突然动作造成危险。
3. 确认串口连接到 UART1。
4. 串口助手参数设置为：
   UART1
   115200
   8N1

当前 UART1 引脚配置：

```text
TX: UART1_TX_F3
RX: UART1_RX_F2
```

### 1.2 测试 MCU 启动和串口通信

上电后，MCU 应主动发送：

```text
$BOOT,SR5E1E3
```

如果 IMU660RA 初始化成功，还会额外发送：

```text
$IMU,OK
```

如果 IMU 芯片 ID 校验失败或配置失败，则会发送：

```text
$IMU,ERR,CHIP_ID
$IMU,ERR,CONFIG
```

发送状态查询命令：

```text
$GET,STATUS
```

正常应返回类似：

```text
$STATUS,mode=DISABLE,state=RUN,tgt_l=0,tgt_r=0,rpm_l=0,rpm_r=0,pwm_l=0,pwm_r=0,bat=7.40,fault=0x00000000
```

如果没有返回，优先检查：

1. 串口是否选对。
2. 波特率是否为 115200。
3. 是否发送了正确包尾 `\r\n`。
4. TX/RX 是否交叉连接。
5. 当前工程是否已经烧录为最新版本。

### 1.3 测试 PWM 开环输出

进入 PWM 开环模式：

```text
$MODE,PWM
```

发送低占空比 PWM：

```text
$PWM,1000,1000
```

用示波器检查 PWM 输出。

正向输出时，预期：

```text
左电机一路高电平，另一路输出 PWM：PE2 / PE4
右电机一路高电平，另一路输出 PWM：PD8 / PD13
```

反向测试：

```text
$PWM,-1000,-1000
```

反向输出时，预期：

```text
左电机与正向相反：PE2 / PE4 中高电平与 PWM 的通道互换
右电机与正向相反：PD8 / PD13 中高电平与 PWM 的通道互换
```

停止输出：

```text
$PWM,0,0
$STOP
```

当前电机驱动按 AT8236 双输入逻辑实现：

```text
运动时：一路保持高电平，另一路输出 PWM
刹车时：两路都输出高电平
```

### 1.4 测试编码器

手动转动车轮，然后发送：

```text
$ENC,GET
```

也可以开启状态周期回传：

```text
$SET,REPORT,1,500
```

观察返回中的：

```text
rpm_l
rpm_r
```

如果手动转动车轮时 RPM 没变化，检查编码器接线和定时器通道配置。

如果 RPM 符号方向反了，需要调整参数或配置中的编码器方向：

```text
left_encoder_dir
right_encoder_dir
```

关闭周期回传：

```text
$SET,REPORT,0,500
```

### 1.5 测试速度闭环

进入速度闭环模式：

```text
$MODE,SPEED
```

发送较低目标速度：

```text
$RPM,100,100
```

查询状态：

```text
$GET,STATUS
```

重点观察：

```text
tgt_l / tgt_r   目标 RPM
rpm_l / rpm_r   实际 RPM
pwm_l / pwm_r   当前 PWM 输出
fault           故障码
```

停止：

```text
$RPM,0,0
$STOP
```

### 1.6 测试底盘 v/w 控制

进入底盘 v/w 模式：

```text
$MODE,VW
```

低速直行：

```text
$VW,0.1,0.0
```

原地转向：

```text
$VW,0.0,0.3
```

停止：

```text
$STOP
```

测试 v/w 前，需要先确认：

1. 左右电机方向正确。
2. 左右编码器方向正确。
3. 速度闭环已经能基本稳定。
4. `wheel_diameter` 和 `wheel_base` 参数接近实际值。

### 1.7 测试 IMU660RA 和 yaw 解算

当前 IMU660RA 使用 I2C 通信，接线为：

```text
SCL: D9
SDA: E8
I2C 地址: 0x69
```

上电后，IMU 初始化成功时不会周期打印原始数据。需要主动查询：

```text
$GET,IMU
```

正常返回类似：

```text
$ATT,yaw=0.00,gz=0.00,bias=-2.50,cal=1
```

字段含义：

```text
yaw  当前 yaw 角度，单位 deg，范围约 -180 ~ +180
gz   去零偏后的 Z 轴角速度，单位 deg/s
bias Z 轴陀螺仪零偏原始值
cal  1 表示零偏校准完成，0 表示仍在校准
```

手动转动车体时，当前实测 yaw 方向为：

```text
左转：yaw 变小，例如 -90
右转：yaw 变大，例如 +90
```

如果需要把当前方向设为 0 度，发送：

```text
$YAW,RESET
```

正常返回：

```text
$OK,YAW
```

注意：IMU 上电后需要短时间静止完成零偏校准。`cal=1` 前，航向控制命令会返回 `$ERR,IMU_NOT_READY`。

### 1.8 测试航向保持直行和指定角度旋转

航向控制复用底盘 `VW` 模式，因此需要先进入：

```text
$MODE,VW
```

建议先复位 yaw：

```text
$YAW,RESET
```

指定角度旋转：

```text
$TURN,90
$TURN,-90
```

当前约定：

```text
$TURN,90   右转 90 度
$TURN,-90  左转 90 度
```

航向保持直行：

```text
$LINE,0.2
```

`$LINE,v` 的含义是：记录当前 yaw 为目标航向，然后以 `v` m/s 前进，同时根据 yaw 偏差自动修正方向。

如果发送：

```text
$LINE,0
```

它不是普通停止，而是“原地保持当前朝向”：线速度为 0，但如果车体被外力转偏，仍会尝试回正。真正停止请使用：

```text
$STOP
```

当前 `$LINE` 航向修正规则：

```text
yaw 误差 <= 0.5 度：不修正
yaw 误差 > 0.5 度：按 P 控制输出角速度 w
如果 w 太小，则至少输出 0.10 rad/s 用于克服静摩擦
```

相关参数在 `user_code/config/board_config.h`：

```c
HEADING_TURN_KP             = 0.04f
HEADING_LINE_KP             = 0.03f
HEADING_TURN_MAX_W_RADPS    = 1.5f
HEADING_LINE_MAX_W_RADPS    = 1.0f
HEADING_TURN_MIN_W_RADPS    = 0.18f
HEADING_TURN_DONE_DEG       = 2.0f
HEADING_LINE_MAX_V_MPS      = 2.0f
HEADING_LINE_DEADBAND_DEG   = 0.5f
HEADING_LINE_MIN_W_RADPS    = 0.10f
HEADING_YAW_TO_W_DIR        = -1.0f
```

航向控制链路：

```text
yaw 误差
  -> heading_control 计算修正角速度 w
  -> chassis 根据 v/w 解算左右轮目标 RPM
  -> motor_control 速度闭环输出 PWM
```

## 2. PID 参数调试流程

### 2.1 修改 PID 参数

PID 参数只能在参数配置模式下修改。

进入参数配置模式：

```text
$MODE,CONFIG
```

设置左轮 PID：

```text
$SET,PID,L,kp,ki,kd
```

设置右轮 PID：

```text
$SET,PID,R,kp,ki,kd
```

例如：

```text
$SET,PID,L,0.5,0.02,0.0
$SET,PID,R,0.5,0.02,0.0
```

当前第一版控制器主要使用 PI 控制，`kd` 可以先保持 `0.0`。

保存参数：

```text
$SAVE
```

注意：当前 Flash 参数读写已经实现。`$SAVE` 会把当前参数写入 DFlash，重新上电后 `param_load()` 会自动读取 Flash 参数；如果 magic、version 或 checksum 校验失败，则自动恢复默认参数。

### 2.2 推荐初始值

代码默认值（`param.c` 中 `car_param_default`）：

```text
kp = 1.0
ki = 0.02
kd = 0.0
```

如果从零开始调试，建议从较小参数开始：

```text
kp = 0.3 ~ 0.8
ki = 0.01 ~ 0.05
kd = 0.0
```

示例：

```text
$MODE,CONFIG
$SET,PID,L,14,1.1,0.0
$SET,PID,R,14,1.1,0.0
$MODE,SPEED
$RPM,60,60
$RPM,0,0
```

### 2.3 调试步骤

1. 先让车轮悬空。
2. 设置较小目标速度，例如：

```text
$MODE,SPEED
$RPM,30,30
```

3. 查询状态：

```text
$GET,STATUS
```

4. 观察目标 RPM、实际 RPM 和 PWM：

```text
tgt_l / tgt_r
rpm_l / rpm_r
pwm_l / pwm_r
```

5. 如果实际 RPM 明显低于目标值，并且 PWM 没有明显震荡，可以逐步增大 `kp`。
6. 如果实际 RPM 接近目标值但长期有稳态误差，可以逐步增大 `ki`。
7. 如果速度明显震荡，先减小 `kp`，再减小 `ki`。
8. 如果 PWM 很快打满但 RPM 不上升，检查电机方向、编码器方向、电源、电机驱动和机械负载。

### 2.4 判断异常现象

电机越控越快或反向加速：

```text
可能是编码器方向反了，或者电机方向和编码器方向不匹配。
```

PWM 输出很大但 RPM 接近 0：

```text
可能是编码器没有读数、电机未接好、驱动未使能或电源不足。
```

目标 RPM 为正，实际 RPM 为负：

```text
编码器方向大概率反了。
```

左右轮同样参数表现差异很大：

```text
检查左右电机接线、编码器接线、轮子阻力、驱动通道和 PID 参数是否一致。
```

### 2.5 Flash 参数保存测试

当前参数保存使用片内 DFlash：

```text
Flash 区域: USER_FLASH_BASE_ADDR = 0x08F00000
使用位置: sector 0 / page 0
保存触发: 只在收到 $SAVE 命令时写入
加载触发: 上电初始化 param_load() 时读取
校验方式: magic + version + checksum
```

推荐用周期状态回传测试 Flash 是否生效，因为它上电后最容易观察。

1. 设置一个容易观察的参数：

```text
$SET,REPORT,1,500
```

2. 进入配置模式并保存：

```text
$MODE,CONFIG
$SAVE
```

3. 断电重启。

4. 如果 Flash 读取成功，上电后除了启动信息：

```text
$BOOT,SR5E1E3
```

还应该每 500ms 自动返回：

```text
$STATUS,...
```

5. 恢复默认参数并保存：

```text
$MODE,CONFIG
$RESET_PARAM
$SAVE
```

6. 再次断电重启。此时应只看到启动信息，不再自动周期回传状态。

注意事项：

```text
$SET,... 只修改 RAM 中的当前参数
$SAVE 才会真正写入 Flash
$LOAD 会重新从 Flash 读取参数
$RESET_PARAM 只恢复 RAM 中的默认参数，想掉电保持默认值还需要再执行 $SAVE
```

## 3. UART 通讯数据包格式规则

### 3.1 串口基础参数

```text
串口: UART1
波特率: 115200
数据位: 8
校验位: None
停止位: 1
格式: 8N1
```

### 3.2 数据包格式

每帧必须满足：

```text
以 $ 开头
以 \r\n 结尾
字段用英文逗号 , 分隔
```

注意：当前固件开启了 **UART RX 回显**。在串口助手中发送的每个字符都会被原样返回，这是正常现象，不影响命令解析。如果不想看到回显，可以在串口助手中关闭本地回显。`board_config.h` 中 `CHASSIS_UART_RX_ECHO_TEST` 控制此功能。

命令解析在 `task_20ms()`（20ms 周期任务）中执行，因此从发送命令到收到响应最多有 20ms 的延迟，属于正常现象。

通用格式：

```text
$CMD,ARG1,ARG2,...\r\n
```

如果串口助手支持自动添加回车换行，可以在发送框中输入：

```text
$GET,STATUS
```

然后勾选发送新行，确保发送的新行是 `\r\n`。

如果串口助手不自动添加包尾，则实际发送内容必须是：

```text
$GET,STATUS\r\n
```

### 3.3 通用返回

执行成功：

```text
$OK,CMD\r\n
```

参数错误：

```text
$ERR,PARAM_RANGE\r\n
```

模式不匹配：

```text
$ERR,MODE_NOT_MATCH\r\n
```

命令无效：

```text
$ERR,CMD_INVALID\r\n
```

### 3.4 模式命令

```text
$MODE,DISABLE\r\n
$MODE,PWM\r\n
$MODE,SPEED\r\n
$MODE,VW\r\n
$MODE,TEST\r\n
$MODE,CONFIG\r\n
```

### 3.5 PWM 开环命令

PWM 范围：

```text
-10000 ~ +10000
```

命令：

```text
$PWM,left_pwm,right_pwm\r\n
$PWM,10000,10000
```

示例：

```text
$MODE,PWM\r\n
$PWM,1000,1000\r\n
$PWM,-1000,-1000\r\n
$PWM,0,0\r\n
```

### 3.6 舵机控制命令

舵机索引范围：1 ~ 2
角度范围：0 ~ 180 度

命令：

```text
$SERVO,index,angle_deg\r\n
```

示例：

```text
$SERVO,1,90\r\n
$SERVO,2,45\r\n
```

`$SERVO` 不需要特定模式，可在任何状态下执行。

### 3.7 速度闭环命令

命令：

```text
$RPM,left_rpm,right_rpm\r\n
```

示例：

```text
$MODE,SPEED\r\n
$RPM,60,60\r\n
$RPM,0,0\r\n
```

`$RPM` 只能在 `SPEED` 模式下执行。

### 3.8 底盘 v/w 命令

单位：

```text
v: m/s
w: rad/s
```

命令：

```text
$VW,v,w\r\n
```

示例：

```text
$MODE,VW\r\n
$VW,0.1,0.0\r\n
$VW,0.0,0.3\r\n
```

`$VW` 只能在 `VW` 模式下执行。

### 3.9 状态和编码器查询

状态查询：

```text
$GET,STATUS\r\n
```

状态返回：

```text
$STATUS,mode=MODE,state=STATE,tgt_l=0,tgt_r=0,rpm_l=0,rpm_r=0,pwm_l=0,pwm_r=0,bat=0.00,fault=0x00000000\r\n
```

编码器查询：

```text
$ENC,GET\r\n
```

编码器返回：

```text
$ENC,left_count,right_count\r\n
```

周期状态回传：

```text
$SET,REPORT,enable,period_ms\r\n
```

示例：

```text
$SET,REPORT,1,500\r\n
$SET,REPORT,0,500\r\n
```

### 3.10 IMU 和航向控制命令

查询 IMU/yaw 状态：

```text
$GET,IMU\r\n
```

返回：

```text
$ATT,yaw=0.00,gz=0.00,bias=0.00,cal=1\r\n
```

复位 yaw：

```text
$YAW,RESET\r\n
```

指定角度旋转，只能在 `VW` 模式下执行：

```text
$TURN,angle_deg\r\n
```

范围：

```text
-180.0 ~ +180.0 deg
```

示例：

```text
$MODE,VW\r\n
$YAW,RESET\r\n
$TURN,90\r\n
$TURN,-90\r\n
```

航向保持直行，只能在 `VW` 模式下执行：

```text
$LINE,v\r\n
```

`v` 单位为 `m/s`，范围受当前参数 `max_v_mps` 限制。

示例：

```text
$MODE,VW\r\n
$LINE,0.2\r\n
$LINE,0\r\n
$STOP\r\n
```

如果 IMU 未在线或零偏校准未完成，会返回：

```text
$ERR,IMU_NOT_READY\r\n
```

### 3.11 故障命令

查询故障：

```text
$FAULT,GET\r\n
```

清除故障：

```text
$FAULT,CLEAR\r\n
```

严重低电压未恢复时，不允许清除故障。

### 3.12 参数配置命令

进入配置模式：

```text
$MODE,CONFIG\r\n
```

PID：

```text
$SET,PID,L,kp,ki,kd\r\n
$SET,PID,R,kp,ki,kd\r\n
```

轮径和轴距：

```text
$SET,WHEEL,wheel_diameter,wheel_base\r\n
```

最大 RPM：

```text
$SET,MAX_RPM,value\r\n
```

最大线速度：

```text
$SET,MAX_V,value\r\n
```

最大角速度：

```text
$SET,MAX_W,value\r\n
```

电池参数：

```text
$SET,BAT,low_warn,low_stop,divider_ratio\r\n
```

保存、加载、恢复默认参数：

```text
$SAVE\r\n
$LOAD\r\n
$RESET_PARAM\r\n
```

### 3.13 安全配置命令

通信超时：

```text
$SET,TIMEOUT,enable,timeout_ms\r\n
```

示例：

```text
$SET,TIMEOUT,1,500\r\n
$SET,TIMEOUT,0,500\r\n
```

故障总开关：

```text
$SET,FAULT,enable\r\n
```

单项故障开关：

```text
$SET,FAULT_ITEM,LOWBAT,enable\r\n
$SET,FAULT_ITEM,ENCODER,enable\r\n
$SET,FAULT_ITEM,BLOCK,enable\r\n
$SET,FAULT_ITEM,COMM,enable\r\n
```

### 3.14 停止命令

```text
$STOP\r\n
```

执行后会停止电机、退出航向控制、清空控制目标并复位 PID 积分。
