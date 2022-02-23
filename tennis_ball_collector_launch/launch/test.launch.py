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
    tennis_court_share = get_package_share_directory("tennis_court")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # Gazebo Server
    gzserver_launch_file = os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
    world_file = os.path.join(tennis_court_share, "worlds", "court.world")
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_file),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": LaunchConfiguration("paused")
        }.items()
    )

    # Gazebo Client
    gzclient_launch_file = os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_file),
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    static_tf_node = Node(
        package="tf2_ros",
        arguments=["0", "0", "8", "3.14159", "1.57079", "3.14159", "map", "zenith_camera_link"],
        executable="static_transform_publisher"
    )

    # Ball Manager
    ball_manager_node = Node(
        package="tennis_court",
        condition=IfCondition(LaunchConfiguration("manager")),
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        executable="ball_manager.py"
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      name='robot_state_publisher',
                                      output='screen',
                                      parameters=[{'robot_description': Command(['xacro ', ' ', urdf_file])},
                                                  {'use_sim_time': use_sim_time}],
                                      arguments=[urdf_file])

    spawn_node = Node(package='gazebo_ros',
                      executable='spawn_entity.py',
                      name='urdf_spawner',
                      output='screen',
                      arguments=["-topic", "/robot_description", "-entity", "robochon", "-y", "3.0", "-z", "0.2"])

    rqt_steering_node = Node(package='rqt_robot_steering',
                             executable='rqt_robot_steering')

    ball_detector_node = Node(package='tennis_ball_detector',
                              executable='detection_node')

    return LaunchDescription([
        DeclareLaunchArgument(name="gui", default_value="true"),
        DeclareLaunchArgument(name="paused", default_value="false"),
        DeclareLaunchArgument(name="manager", default_value="true"),
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        gzserver_launch,
        gzclient_launch,
        static_tf_node,
        ball_manager_node,
        robot_state_publisher_node,
        spawn_node,
        rqt_steering_node,
        ball_detector_node,
    ])
