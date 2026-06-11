#!/usr/bin/env python3
"""
ROS2 话题版小车控制节点。

订阅 std_msgs/msg/String 类型的 car_movement：
  data="0"          -> 停车，发送 $STOP\r\n
  data="1,60"       -> 直行，发送 $MODE,SPEED\r\n 和 $RPM,60,60\r\n
  data="2,0.5,60"  -> 左转 0.5 秒，再恢复直行
  data="3,1.2,60"  -> 右转 1.2 秒，再恢复直行
"""

import argparse
import os
import select
import sys
import termios
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:
    print(
        "ERROR: 需要在 ROS2 环境中运行，请先 source /opt/ros/<distro>/setup.bash",
        file=sys.stderr,
    )
    raise


DEFAULT_PORT = "/dev/ttyS6"
BAUD_RATE = 115200
READ_TIMEOUT_S = 0.05
DEFAULT_SPEED = 60
TOPIC_NAME = "car_movement"

CMD_STOP = "$STOP\r\n"
CMD_SPEED_MODE = "$MODE,SPEED\r\n"


def baud_to_termios(baud):
    """把数字波特率转换为 termios 常量，例如 115200 -> B115200。"""
    constant_name = f"B{baud}"
    if not hasattr(termios, constant_name):
        raise ValueError(f"unsupported baud rate: {baud}")
    return getattr(termios, constant_name)


def configure_uart(fd, baud):
    """配置串口为 MD 要求的 115200 8N1 原始模式。"""
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[3] = 0

    # CLOCAL/CREAD 打开本地串口接收；CS8/PARENB/CSTOPB 对应 8N1。
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS

    baud_flag = baud_to_termios(baud)
    attrs[4] = baud_flag
    attrs[5] = baud_flag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_available(fd, timeout_s):
    """短时间读取驱动板返回，避免 ROS2 回调被长时间阻塞。"""
    deadline = time.monotonic() + timeout_s
    chunks = []

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break

        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            continue

        if not data:
            break
        chunks.append(data)

    return b"".join(chunks)


def make_rpm_frame(left_rpm, right_rpm):
    """生成 MD 中定义的 $RPM,left_rpm,right_rpm\\r\\n 帧。"""
    return f"$RPM,{left_rpm},{right_rpm}\r\n"


class CarMovementNode(Node):
    def __init__(self, port):
        super().__init__("orangepi_to_car")
        self.fd = None
        self.turn_timer = None
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_uart(self.fd, BAUD_RATE)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

        self.current_speed = DEFAULT_SPEED
        self.create_subscription(
            String,
            TOPIC_NAME,
            self.on_movement,
            10,
        )
        self.get_logger().info(f"Opened {port} at {BAUD_RATE} 8N1")
        self.get_logger().info(
            'Topic car_movement String: "0"=STOP, "1,speed"=forward, '
            '"2,time_s,speed"=left, "3,time_s,speed"=right'
        )

    def close(self):
        if self.turn_timer is not None:
            self.turn_timer.cancel()
            self.turn_timer = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def send_command(self, command):
        """发送一帧驱动板协议，所有命令都带有 \\r\\n 包尾。"""
        self.get_logger().info(f"TX: {command.rstrip()!r}")
        os.write(self.fd, command.encode("ascii"))
        termios.tcdrain(self.fd)

        response = read_available(self.fd, READ_TIMEOUT_S)
        if response:
            text = response.decode("ascii", errors="replace").rstrip()
            self.get_logger().info(f"RX: {text!r}")

    def cancel_turn_timer(self):
        if self.turn_timer is not None:
            self.turn_timer.cancel()
            self.turn_timer = None

    def enter_speed_mode(self):
        # MD 要求 $RPM 只能在 SPEED 模式下执行。
        self.send_command(CMD_SPEED_MODE)

    def stop_car(self):
        self.cancel_turn_timer()
        self.send_command(CMD_STOP)

    def forward(self, speed):
        self.cancel_turn_timer()
        self.current_speed = speed
        self.enter_speed_mode()
        self.send_command(make_rpm_frame(speed, speed))

    def start_turn(self, duration_s, speed, direction):
        self.cancel_turn_timer()
        self.current_speed = speed
        self.enter_speed_mode()

        # 左转停左轮，右转停右轮；duration_s 秒后定时恢复直行。
        if direction == "left":
            self.send_command(make_rpm_frame(0, speed))
        else:
            self.send_command(make_rpm_frame(speed, 0))

        self.turn_timer = self.create_timer(duration_s, self.finish_turn)

    def finish_turn(self):
        self.cancel_turn_timer()
        self.send_command(make_rpm_frame(self.current_speed, self.current_speed))

    def parse_message(self, msg):
        fields = [field.strip() for field in msg.data.split(",")]
        if not fields or not fields[0]:
            raise ValueError("empty data")

        command = int(fields[0])
        if command == 0:
            if len(fields) != 1:
                raise ValueError("stop command format must be 0")
            return command, None, None

        if command == 1:
            if len(fields) > 2:
                raise ValueError("forward command format must be 1 or 1,speed")
            speed = DEFAULT_SPEED if len(fields) < 2 else int(fields[1])
            return command, None, speed

        if command in (2, 3):
            if len(fields) != 3:
                raise ValueError(
                    "turn command format must be 2,time_s,speed "
                    "or 3,time_s,speed"
                )
            duration_s = float(fields[1])
            speed = int(fields[2])
            if duration_s <= 0:
                raise ValueError("turn time must be > 0")
            return command, duration_s, speed

        return command, None, None

    def on_movement(self, msg):
        try:
            command, duration_s, speed = self.parse_message(msg)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid message: {exc}")
            return

        if command == 0:
            self.stop_car()
        elif command == 1:
            self.forward(speed)
        elif command == 2:
            self.start_turn(duration_s, speed, "left")
        elif command == 3:
            self.start_turn(duration_s, speed, "right")
        else:
            self.get_logger().warning(
                f"Unknown command {command}; no UART command was sent"
            )


def parse_args(args=None):
    parser = argparse.ArgumentParser(description="ROS2 car_movement UART bridge.")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"串口设备路径，默认: {DEFAULT_PORT}",
    )
    # parse_known_args 保留 ROS2 自己的 --ros-args / remap 参数。
    return parser.parse_known_args(args)


def main(argv=None):
    parsed_args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args if ros_args else None)
    node = None

    try:
        node = CarMovementNode(parsed_args.port)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f"ERROR: cannot open/use {parsed_args.port}: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
