from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import rclpy
from rclpy.node import Node


VALID_ACTIONS = {"start", "stop", "status"}


class CommandError(RuntimeError):
    def __init__(self, command: Sequence[str], result: subprocess.CompletedProcess[str]):
        self.command = command
        self.result = result
        stderr = result.stderr.strip()
        stdout = result.stdout.strip()
        detail = stderr or stdout or f"exit code {result.returncode}"
        super().__init__(f"{' '.join(command)} failed: {detail}")


@dataclass(frozen=True)
class ApConfig:
    action: str
    ssid: str
    interface: str
    connection_name: str
    ip_cidr: str
    band: str
    channel: int


def run_command(command: Sequence[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        list(command),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    if check and result.returncode != 0:
        raise CommandError(command, result)
    return result


def clean_lines(output: str) -> list[str]:
    return [line.strip() for line in output.splitlines() if line.strip()]


class ApManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("wifi_ap_manager")
        self.declare_parameter("action", "start")
        self.declare_parameter("ssid", "OPi_ROS2_TEST")
        self.declare_parameter("interface", "wlan0")
        self.declare_parameter("connection_name", "OPi_ROS2_OPEN_AP")
        self.declare_parameter("ip_cidr", "192.168.50.1/24")
        self.declare_parameter("band", "bg")
        self.declare_parameter("channel", 6)

    def execute(self) -> None:
        config = self.read_config()
        if config.action == "start":
            self.start_ap(config)
        elif config.action == "stop":
            self.stop_ap(config)
        elif config.action == "status":
            self.show_status(config)
        else:
            actions = ", ".join(sorted(VALID_ACTIONS))
            raise ValueError(f"Unsupported action '{config.action}'. Use one of: {actions}")

    def read_config(self) -> ApConfig:
        action = self.get_parameter("action").value
        ssid = self.get_parameter("ssid").value
        interface = self.get_parameter("interface").value
        connection_name = self.get_parameter("connection_name").value
        ip_cidr = self.get_parameter("ip_cidr").value
        band = self.get_parameter("band").value
        channel = self.get_parameter("channel").value

        config = ApConfig(
            action=str(action).strip().lower(),
            ssid=str(ssid).strip(),
            interface=str(interface).strip(),
            connection_name=str(connection_name).strip(),
            ip_cidr=str(ip_cidr).strip(),
            band=str(band).strip(),
            channel=int(channel),
        )
        self.validate_config(config)
        return config

    def validate_config(self, config: ApConfig) -> None:
        if not config.ssid:
            raise ValueError("Parameter 'ssid' must not be empty")
        if not config.interface:
            raise ValueError("Parameter 'interface' must not be empty")
        if not config.connection_name:
            raise ValueError("Parameter 'connection_name' must not be empty")
        if "/" not in config.ip_cidr:
            raise ValueError("Parameter 'ip_cidr' must include a prefix, for example 192.168.50.1/24")
        if config.band not in {"bg", "a"}:
            raise ValueError("Parameter 'band' must be 'bg' for 2.4GHz or 'a' for 5GHz")
        if not 1 <= config.channel <= 196:
            raise ValueError("Parameter 'channel' must be in the range 1..196")

    def start_ap(self, config: ApConfig) -> None:
        self.ensure_prerequisites(config)
        if not self.connection_exists(config.connection_name):
            self.get_logger().info(f"Creating NetworkManager connection '{config.connection_name}'")
            run_command(
                [
                    "nmcli",
                    "connection",
                    "add",
                    "type",
                    "wifi",
                    "ifname",
                    config.interface,
                    "con-name",
                    config.connection_name,
                    "autoconnect",
                    "no",
                    "ssid",
                    config.ssid,
                    "mode",
                    "ap",
                ]
            )

        self.get_logger().info(
            f"Configuring open AP '{config.ssid}' on {config.interface} at {config.ip_cidr}"
        )
        run_command(
            [
                "nmcli",
                "connection",
                "modify",
                config.connection_name,
                "connection.interface-name",
                config.interface,
                "connection.autoconnect",
                "no",
                "802-11-wireless.mode",
                "ap",
                "802-11-wireless.ssid",
                config.ssid,
                "802-11-wireless.band",
                config.band,
                "802-11-wireless.channel",
                str(config.channel),
                "ipv4.method",
                "shared",
                "ipv4.addresses",
                config.ip_cidr,
                "ipv6.method",
                "disabled",
            ]
        )
        self.remove_wifi_security(config.connection_name)

        self.get_logger().warn(
            "Starting the AP will disconnect this Wi-Fi interface from its current network."
        )
        run_command(["nmcli", "connection", "up", config.connection_name])
        self.get_logger().info(
            f"Open hotspot is active: SSID='{config.ssid}', interface={config.interface}, ip={config.ip_cidr}"
        )
        self.get_logger().info("Phones and Windows may show 'No Internet'; that is expected.")

    def stop_ap(self, config: ApConfig) -> None:
        self.ensure_nmcli()
        if not self.connection_exists(config.connection_name):
            self.get_logger().info(f"Connection '{config.connection_name}' does not exist; nothing to stop")
            return
        if not self.connection_active(config.connection_name):
            self.get_logger().info(f"Connection '{config.connection_name}' is already inactive")
            return

        run_command(["nmcli", "connection", "down", config.connection_name])
        self.get_logger().info(f"Stopped hotspot connection '{config.connection_name}'")
        self.get_logger().info("NetworkManager may reconnect a saved Wi-Fi network automatically.")

    def show_status(self, config: ApConfig) -> None:
        self.ensure_nmcli()
        self.get_logger().info(f"nmcli: {shutil.which('nmcli')}")
        self.log_command_output(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
        self.log_command_output(["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"])
        self.log_command_output(["ip", "-br", "addr", "show", config.interface], check=False)

        exists = self.connection_exists(config.connection_name)
        active = self.connection_active(config.connection_name) if exists else False
        self.get_logger().info(
            f"hotspot_connection={config.connection_name} exists={exists} active={active}"
        )

    def ensure_prerequisites(self, config: ApConfig) -> None:
        self.ensure_nmcli()
        if not Path("/sys/class/net", config.interface).exists():
            raise RuntimeError(f"Interface '{config.interface}' was not found")

        wifi_state = run_command(["nmcli", "radio", "wifi"]).stdout.strip().lower()
        if wifi_state != "enabled":
            raise RuntimeError("Wi-Fi radio is disabled. Enable it first with: nmcli radio wifi on")

    def ensure_nmcli(self) -> None:
        if shutil.which("nmcli") is None:
            raise RuntimeError("nmcli was not found. Install or enable NetworkManager first.")

    def connection_exists(self, connection_name: str) -> bool:
        result = run_command(["nmcli", "-g", "NAME", "connection", "show"])
        return connection_name in clean_lines(result.stdout)

    def connection_active(self, connection_name: str) -> bool:
        result = run_command(["nmcli", "-g", "NAME", "connection", "show", "--active"])
        return connection_name in clean_lines(result.stdout)

    def remove_wifi_security(self, connection_name: str) -> None:
        result = run_command(
            [
                "nmcli",
                "connection",
                "modify",
                connection_name,
                "remove",
                "802-11-wireless-security",
            ],
            check=False,
        )
        if result.returncode == 0:
            return

        message = f"{result.stdout}\n{result.stderr}".lower()
        if "not present" in message or "no such setting" in message:
            return
        raise CommandError(
            [
                "nmcli",
                "connection",
                "modify",
                connection_name,
                "remove",
                "802-11-wireless-security",
            ],
            result,
        )

    def log_command_output(self, command: Iterable[str], *, check: bool = True) -> None:
        command_list = list(command)
        result = run_command(command_list, check=check)
        output = result.stdout.strip() or result.stderr.strip() or "(no output)"
        self.get_logger().info(f"$ {' '.join(command_list)}\n{output}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ApManagerNode()
    exit_code = 0
    try:
        node.execute()
    except Exception as exc:  # noqa: BLE001 - this is a command-line ROS helper.
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
