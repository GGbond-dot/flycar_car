from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("action", default_value="start"),
            DeclareLaunchArgument("ssid", default_value="OPi_ROS2_TEST"),
            DeclareLaunchArgument("interface", default_value="wlan0"),
            DeclareLaunchArgument("connection_name", default_value="OPi_ROS2_OPEN_AP"),
            DeclareLaunchArgument("ip_cidr", default_value="192.168.50.1/24"),
            DeclareLaunchArgument("band", default_value="bg"),
            DeclareLaunchArgument("channel", default_value="6"),
            Node(
                package="wifi_ap_manager",
                executable="ap_manager",
                name="wifi_ap_manager",
                output="screen",
                parameters=[
                    {
                        "action": ParameterValue(LaunchConfiguration("action"), value_type=str),
                        "ssid": ParameterValue(LaunchConfiguration("ssid"), value_type=str),
                        "interface": ParameterValue(LaunchConfiguration("interface"), value_type=str),
                        "connection_name": ParameterValue(
                            LaunchConfiguration("connection_name"), value_type=str
                        ),
                        "ip_cidr": ParameterValue(LaunchConfiguration("ip_cidr"), value_type=str),
                        "band": ParameterValue(LaunchConfiguration("band"), value_type=str),
                        "channel": ParameterValue(LaunchConfiguration("channel"), value_type=int),
                    }
                ],
            ),
        ]
    )
