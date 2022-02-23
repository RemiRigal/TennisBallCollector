import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, LoadComposableNodes
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory('tennis_ball_collector_launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'urdf/robochon.urdf.xacro'
    urdf_file = os.path.join(get_package_share_directory('robochon_description'), urdf_file_name)
    nav2_params = os.path.join(get_package_share_directory('robochon_description'), 'config', 'nav2_params.yaml')
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'map_server',
                       'amcl']
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                         default_value='false',
                                         description='Use simulation (Gazebo) clock if true')

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      parameters=[{'robot_description': Command(['xacro ', ' ', urdf_file])},
                                                  {'use_sim_time': use_sim_time}],
                                      arguments=[urdf_file])

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('robochon_description'), 'config/ekf.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    recoveries_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    bt_nav_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    map_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params],
        remappings=remappings)
    lf_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    spawn_node = Node(package='gazebo_ros',
                      executable='spawn_entity.py',
                      name='urdf_spawner',
                      output='screen',
                      arguments=["-topic", "/robot_description", "-entity", "robochon", "-y", "3.0", "-z", "0.2"])

    return LaunchDescription([
        sim_time_arg,
        robot_state_publisher_node,
        spawn_node,
        # robot_localization_node,
        controller_node,
        # smoother_node,
        planner_node,
        recoveries_node,
        bt_nav_node,
        waypoint_node,
        amcl_node,
        map_node,
        lf_manager_node,
    ])
