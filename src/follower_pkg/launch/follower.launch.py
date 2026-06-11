"""启动跟随三节点:leader_pose_receiver + follower_node + diff_drive_controller,
参数来自 config/follower_params.yaml。"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('follower_pkg').find('follower_pkg')
    params_file = os.path.join(pkg_share, 'config', 'follower_params.yaml')

    return LaunchDescription([
        Node(
            package='follower_pkg',
            executable='leader_pose_receiver',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='follower_pkg',
            executable='follower_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='follower_pkg',
            executable='diff_drive_controller',
            parameters=[params_file],
            output='screen',
        ),
    ])
