import launch
from launch.substitutions import Command, LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robochon_description').find('robochon_description')
    default_model_path = os.path.join(pkg_share, 'urdf/robochon.urdf.xacro')
    
    pkg_share_world = launch_ros.substitutions.FindPackageShare(package='robochon_description').find('tennis_court')
    world_path=os.path.join(pkg_share_world, 'world/court.world')
    

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'robochon', '-topic', 'robot_description',"-y","3"],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    rqt_robot_steering = launch_ros.actions.Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering',
            output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',description='Flag to enable joint_state_publisher_gui'),
        
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,description='Absolute path to robot urdf file'),
        
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',description='Flag to enable use_sim_time'),
        
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        #joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        #rqt_robot_steering,
    ])
