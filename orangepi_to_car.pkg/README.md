# orangepi_to_car

ROS2 Python 功能包，用于把 `/car_movement` 话题转换为 SR5E1E3 底盘串口协议。

当前包只保留 `orangepi_to_carv2` 入口，使用航向保持、定角转向和舵机初始化逻辑。

## 话题格式

节点订阅：

```text
/car_movement std_msgs/msg/String
```

消息格式：

```text
"0"      停车
"1"      航向保持直行，默认 0.5 m/s
"1,v"    航向保持直行，自定义速度 v，单位 m/s
"2"      左转 90 度
"3"      右转 90 度
"4"      右转掉头 180 度
"5"      1 号舵机扫动，90 -> 160 -> 90
```

节点启动时会先反复发送：

```text
$SERVO,1,160\r\n
```

只有收到 `$OK` 后才开始订阅并执行 `/car_movement` 话题。

## 构建

在工作区根目录执行：

```bash
colcon build --symlink-install --packages-select orangepi_to_car
source install/setup.bash
```

如果当前目录就是 `/home/orangepi/flycar_d/car`，可以直接把这里当工作区根目录：

```bash
cd /home/orangepi/flycar_d/car
colcon build --symlink-install --packages-select orangepi_to_car
source install/setup.bash
```

## 运行

```bash
ros2 run orangepi_to_car orangepi_to_carv2 --port /dev/ttyS6 --baud 115200
```

也可以用 launch。

```bash
ros2 launch orangepi_to_car orangepi_to_carv2.launch.py port:=/dev/ttyS6 baud:=115200
```

## 模拟发送一次话题

```bash
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '1'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '1,0.5'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '2'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '3'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '4'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '5'}"
ros2 topic pub --once /car_movement std_msgs/msg/String "{data: '0'}"
```
