#!/usr/bin/env python3
"""
SR5E1E3 小车底盘 ROS2 话题控制节点 v2。

订阅 std_msgs/msg/String 类型的 car_movement：
  data="0"    -> 停车，发送 $STOP
  data="1"    -> 航向保持直行，发送 $GET,IMU + $MODE,VW + $LINE,0.5
  data="1,v"  -> 航向保持直行，发送 $GET,IMU + $MODE,VW + $LINE,v
  data="2"    -> 左转 90 度，发送 $GET,IMU + $MODE,VW + $TURN,-90
  data="3"    -> 右转 90 度，发送 $GET,IMU + $MODE,VW + $TURN,90
  data="4"    -> 右转掉头 180 度，发送 $GET,IMU + $MODE,VW + $TURN,180
  data="5"    -> 1 号舵机扫动，发送 $SERVO,1,90 -> $SERVO,1,160 -> $SERVO,1,90

本脚本只发送 SR5E1E3_CHASSIS_DEBUG_GUIDE.md 中已经写明的核心串口帧。
启动时会反复发送 $SERVO,1,160，直到收到 $OK 后再接受话题。
"""

import argparse
import os
import re
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
DEFAULT_BAUD = 115200
TOPIC_NAME = "car_movement"

# 普通命令读取时间可以短一些；IMU 查询稍长，避免偶发读不到完整 $ATT。
READ_TIMEOUT_S = 0.35
IMU_READ_TIMEOUT_S = 0.6

# $LINE 的 v 单位是 m/s，范围与固件 VW/航向控制调试值保持一致。
DEFAULT_LINE_V = 0.50
LINE_MIN_V = -2.0
LINE_MAX_V = 2.0

# 新版 MD 定义舵机口 1~2，角度 0~180 度；这里只保留 1 号舵机扫动。
SERVO_TEST_A_DEG = 90
SERVO_TEST_B_DEG = 160
SERVO_SWEEP_DELAY_S = 1.5

CMD_STOP = "$STOP\r\n"
CMD_VW_MODE = "$MODE,VW\r\n"
CMD_IMU = "$GET,IMU\r\n"

# 解析 MD 中定义的 IMU 返回帧，例如：
# $ATT,yaw=0.00,gz=0.00,bias=-2.50,cal=1
ATT_RE = re.compile(
    r"\$ATT,yaw=([-+]?\d+(?:\.\d+)?),"
    r"gz=([-+]?\d+(?:\.\d+)?),"
    r"bias=([-+]?\d+(?:\.\d+)?),"
    r"cal=(\d+)"
)


class CommandResult:
    """一条串口命令的返回结果，供级联动作判断是否继续。"""

    def __init__(self, command, response):
        self.command = command
        self.response = response
        self.text = decode_response(response) if response else ""
        self.status = classify_response(response)

    @property
    def can_continue(self):
        return self.status not in ("NO_RESPONSE", "ERR")


def classify_response(response):
    if not response:
        return "NO_RESPONSE"

    text = decode_response(response)
    if "$ERR," in text:
        return "ERR"
    if "$OK," in text:
        return "OK"
    if "$ATT" in text:
        return "DATA"
    return "RX_ONLY"


def baud_to_termios(baud):
    """把数字波特率转换为 termios 常量，例如 115200 -> B115200。"""
    constant_name = f"B{baud}"
    if not hasattr(termios, constant_name):
        raise ValueError(f"unsupported baud rate: {baud}")
    return getattr(termios, constant_name)


def configure_uart(fd, baud):
    """配置串口为 MD 要求的原始 8N1 模式。"""
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0  # 输入不做换行、控制字符等转换。
    attrs[1] = 0  # 输出不做换行转换。
    attrs[3] = 0  # 关闭本地回显和规范输入，直接收发字节。

    # CLOCAL/CREAD 打开本地串口接收；CS8、无校验、1 停止位对应 8N1。
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
    """读取 timeout_s 内已经到达的所有串口字节。"""
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


def decode_response(response):
    return response.decode("ascii", errors="replace").rstrip()


def send_command(fd, command, read_timeout_s=READ_TIMEOUT_S, logger=None):
    """发送一帧已带 CRLF 包尾的 MD 命令，并读取驱动板返回。"""
    if logger is not None:
        logger.info(f"TX: {command.rstrip()!r}")

    os.write(fd, command.encode("ascii"))
    termios.tcdrain(fd)

    response = read_available(fd, read_timeout_s)
    result = CommandResult(command, response)

    if logger is not None:
        if response:
            logger.info(f"RX: {decode_response(response)!r} [{result.status}]")
        else:
            logger.info(f"RX: <no response> [{result.status}]")

    return result


def make_frame(*fields):
    """统一生成 '$字段,字段\\r\\n'，避免手写帧时漏掉 CRLF。"""
    return "$" + ",".join(str(field) for field in fields) + "\r\n"


def make_line_frame(v):
    return make_frame("LINE", format_float(v))


def make_turn_frame(angle_deg):
    return make_frame("TURN", format_float(angle_deg))


def make_servo_frame(index, angle_deg):
    return make_frame("SERVO", int(index), int(angle_deg))


def format_float(value):
    """把浮点数压成适合串口调试阅读的短格式，例如 0.300 -> 0.3。"""
    return f"{float(value):.3f}".rstrip("0").rstrip(".")


def parse_att_response(response):
    """从串口返回中提取最后一帧 $ATT；没有合法帧时返回 None。"""
    text = decode_response(response)
    matches = list(ATT_RE.finditer(text))
    if not matches:
        return None

    match = matches[-1]
    return {
        "yaw": float(match.group(1)),
        "gz": float(match.group(2)),
        "bias": float(match.group(3)),
        "cal": int(match.group(4)),
    }


class CarMovementV2Node(Node):
    def __init__(self, port, baud):
        super().__init__("orangepi_to_carv2")
        self.fd = None
        self.servo_timer = None
        self.servo_sequence = []
        self.servo_step_index = 0

        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            configure_uart(self.fd, baud)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

        self.initialize_servo()

        self.subscription = self.create_subscription(
            String,
            TOPIC_NAME,
            self.on_movement,
            10,
        )
        self.get_logger().info(f"Opened {port} at {baud} 8N1")
        self.get_logger().info(
            'Topic car_movement String: "0"=STOP, "1[,v]"=LINE, '
            '"2"=left90, "3"=right90, "4"=right180, "5"=servo sweep'
        )

    def initialize_servo(self):
        """启动时把 1 号舵机回到 160 度，收到 $OK 后才允许话题控制。"""
        command = make_servo_frame(1, SERVO_TEST_B_DEG)

        while rclpy.ok():
            result = self.send_command(command)
            if result.status == "OK":
                time.sleep(SERVO_SWEEP_DELAY_S)
                return

            self.get_logger().warning(
                "Servo initialization did not receive $OK; retrying"
            )
            time.sleep(SERVO_SWEEP_DELAY_S)

        raise RuntimeError("ROS shutdown before servo initialization received $OK")

    def close(self):
        self.cancel_servo_sweep()
        if self.fd is not None:
            try:
                self.send_command(CMD_STOP)
            except OSError as exc:
                self.get_logger().error(f"Failed to send $STOP on close: {exc}")
            os.close(self.fd)
            self.fd = None

    def send_command(self, command, read_timeout_s=READ_TIMEOUT_S):
        return send_command(
            self.fd,
            command,
            read_timeout_s=read_timeout_s,
            logger=self.get_logger(),
        )

    def read_imu(self):
        result = self.send_command(CMD_IMU, read_timeout_s=IMU_READ_TIMEOUT_S)
        if not result.can_continue:
            return None
        return parse_att_response(result.response)

    def ensure_imu_ready(self):
        imu = self.read_imu()
        if imu is None:
            self.get_logger().warning(
                "Abort: no valid IMU response, so no heading command was sent"
            )
            return False
        if imu["cal"] != 1:
            self.get_logger().warning(
                "Abort: IMU cal=0; keep the car still and retry after calibration"
            )
            return False
        return True

    def enter_vw_mode(self):
        result = self.send_command(CMD_VW_MODE)
        if not result.can_continue:
            self.get_logger().warning(
                "Abort: VW mode was not accepted, so no motion command was sent"
            )
            return False
        return True

    def stop_car(self):
        self.cancel_servo_sweep()
        self.send_command(CMD_STOP)

    def line_forward(self, line_v):
        self.cancel_servo_sweep()
        if not self.ensure_imu_ready():
            return
        if not self.enter_vw_mode():
            return
        self.send_command(make_line_frame(line_v))

    def turn_angle(self, angle_deg, direction_name):
        self.cancel_servo_sweep()
        self.get_logger().info(
            f"Start {direction_name}: $TURN,{format_float(angle_deg)}"
        )
        if not self.ensure_imu_ready():
            return
        if not self.enter_vw_mode():
            return
        self.send_command(make_turn_frame(angle_deg))

    def start_servo_sweep(self):
        self.cancel_servo_sweep()
        self.servo_sequence = [
            SERVO_TEST_A_DEG,
            SERVO_TEST_B_DEG,
            SERVO_TEST_A_DEG,
        ]
        self.servo_step_index = 0
        self.send_next_servo_angle()
        if self.servo_step_index < len(self.servo_sequence):
            self.servo_timer = self.create_timer(
                SERVO_SWEEP_DELAY_S,
                self.send_next_servo_angle,
            )

    def send_next_servo_angle(self):
        if self.servo_step_index >= len(self.servo_sequence):
            self.cancel_servo_sweep()
            return

        angle = self.servo_sequence[self.servo_step_index]
        self.servo_step_index += 1
        result = self.send_command(make_servo_frame(1, angle))
        if not result.can_continue:
            self.get_logger().warning("Abort: servo command failed or timed out")
            self.cancel_servo_sweep()
            return

        if self.servo_step_index >= len(self.servo_sequence):
            self.cancel_servo_sweep()

    def cancel_servo_sweep(self):
        if self.servo_timer is not None:
            self.servo_timer.cancel()
            self.destroy_timer(self.servo_timer)
            self.servo_timer = None
        self.servo_sequence = []
        self.servo_step_index = 0

    def parse_message(self, data):
        fields = [field.strip() for field in data.split(",")]
        if not fields or not fields[0]:
            raise ValueError("empty data")

        try:
            command = int(fields[0])
        except ValueError as exc:
            raise ValueError("command must be an integer") from exc

        if command == 0:
            if len(fields) != 1:
                raise ValueError('stop command format must be "0"')
            return command, None

        if command == 1:
            if len(fields) > 2:
                raise ValueError('line command format must be "1" or "1,v"')
            if len(fields) == 1:
                line_v = DEFAULT_LINE_V
            else:
                if not fields[1]:
                    raise ValueError("line speed must not be empty")
                line_v = float(fields[1])
            if line_v < LINE_MIN_V or line_v > LINE_MAX_V:
                raise ValueError(
                    f"line speed must be in [{LINE_MIN_V}, {LINE_MAX_V}] m/s"
                )
            return command, line_v

        if command in (2, 3, 4, 5):
            if len(fields) != 1:
                raise ValueError(f'command {command} format must be "{command}"')
            return command, None

        raise ValueError(f"unknown command {command}")

    def on_movement(self, msg):
        try:
            command, line_v = self.parse_message(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f"Invalid message: {exc}")
            return

        if command == 0:
            self.stop_car()
        elif command == 1:
            self.line_forward(line_v)
        elif command == 2:
            self.turn_angle(-90.0, "left 90 deg")
        elif command == 3:
            self.turn_angle(90.0, "right 90 deg")
        elif command == 4:
            self.turn_angle(180.0, "right 180 deg")
        elif command == 5:
            self.start_servo_sweep()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="ROS2 car_movement UART bridge v2.")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"UART device path, default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"UART baud rate, default: {DEFAULT_BAUD}",
    )
    # parse_known_args 保留 ROS2 自己的 --ros-args / remap 参数。
    return parser.parse_known_args(argv)


def main(argv=None):
    parsed_args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args if ros_args else None)
    node = None
    exit_code = 0

    try:
        node = CarMovementV2Node(parsed_args.port, parsed_args.baud)
        rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
    except OSError as exc:
        print(f"ERROR: cannot open/use {parsed_args.port}: {exc}", file=sys.stderr)
        exit_code = 1
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
