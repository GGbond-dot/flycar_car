"""car 跟随任务总入口:建图定位 + 跟随三节点 + 底盘桥。

底盘桥以 chassis_timeout_ms=500 启动(底盘侧通信超时兜底,跟随场景专用;
语音/离散命令场景请用 orangepi_to_car 自己的 launch,默认不开超时)。
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _include_launch(package_name: str, filename: str, launch_arguments=None) -> IncludeLaunchDescription:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, "launch", filename)),
        launch_arguments=launch_arguments.items() if launch_arguments else None,
    )


def generate_launch_description():
    car_carto_launch = _include_launch("car_carto_pkg", "car_carto.launch.py")
    follower_launch = _include_launch("follower_pkg", "follower.launch.py")
    chassis_launch = _include_launch(
        "orangepi_to_car", "orangepi_to_carv2.launch.py",
        launch_arguments={"chassis_timeout_ms": "500"},
    )

    return LaunchDescription([
        car_carto_launch,
        TimerAction(
            period=3.0,
            actions=[
                follower_launch,
                chassis_launch,
            ],
        ),
    ])
