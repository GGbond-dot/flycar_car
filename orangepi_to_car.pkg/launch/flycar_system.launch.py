from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    terminal_root = LaunchConfiguration("terminal_root")
    terminal_mode = LaunchConfiguration("terminal_mode")
    terminal_python = LaunchConfiguration("terminal_python")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyS6",
                description="UART device path connected to the chassis controller.",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="UART baud rate used by the chassis controller.",
            ),
            DeclareLaunchArgument(
                "terminal_root",
                default_value=(
                    "/home/orangepi/qianrushi/flycar_d/"
                    "terminal/kian_ai_0001-main"
                ),
                description="Path to the terminal AI agent project root.",
            ),
            DeclareLaunchArgument(
                "terminal_mode",
                default_value="web",
                description="Terminal UI mode: web, gui, or cli.",
            ),
            DeclareLaunchArgument(
                "terminal_python",
                default_value="/home/orangepi/miniconda3/envs/py-xiaozhi/bin/python",
                description="Python executable used to run terminal/main.py.",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="10",
                description="ROS_DOMAIN_ID shared by terminal and car nodes.",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            Node(
                package="orangepi_to_car",
                executable="orangepi_to_carv2",
                name="orangepi_to_carv2",
                output="screen",
                arguments=["--port", port, "--baud", baud],
            ),
            ExecuteProcess(
                cmd=[
                    terminal_python,
                    "main.py",
                    "--mode",
                    terminal_mode,
                    "--protocol",
                    "local",
                ],
                cwd=terminal_root,
                output="screen",
            ),
        ]
    )
