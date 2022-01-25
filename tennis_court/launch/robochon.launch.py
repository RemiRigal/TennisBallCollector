import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf_file_name = 'urdf/robochon.urdf.xacro'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('robochon_description'),
      urdf_file_name)

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', ' ', urdf])},{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering'
            ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "robochon"])
  ])