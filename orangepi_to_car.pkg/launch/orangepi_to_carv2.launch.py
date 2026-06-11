from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

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
            Node(
                package="orangepi_to_car",
                executable="orangepi_to_carv2",
                name="orangepi_to_carv2",
                output="screen",
                arguments=["--port", port, "--baud", baud],
            ),
        ]
    )
