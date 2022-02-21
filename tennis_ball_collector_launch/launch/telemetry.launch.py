import os

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tennis_ball_collector_launch")

    # Rviz
    rviz_config_file = os.path.join(pkg_share, "config", "layout.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        # parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        rviz_node,
    ])
