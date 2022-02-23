import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('tennis_ball_collector_launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/robochon.urdf.xacro'
    urdf_file = os.path.join(get_package_share_directory('robochon_description'), urdf_file_name)

    sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                         default_value='true',
                                         description='Use simulation (Gazebo) clock if true')

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      parameters=[{'robot_description': Command(['xacro ', ' ', urdf_file])},
                                                  {'use_sim_time': use_sim_time}],
                                      arguments=[urdf_file])

    ball_detector_node = Node(package='tennis_ball_detector',
                              executable='detection_node',
                              name='detection_node')

    spawn_node = Node(package='gazebo_ros',
                      executable='spawn_entity.py',
                      name='urdf_spawner',
                      output='screen',
                      arguments=["-topic", "/robot_description", "-entity", "robochon", "-y", "3.0", "-z", "0.2"])

    return LaunchDescription([
        sim_time_arg,
        robot_state_publisher_node,
        spawn_node,
        ball_detector_node,

    ])
