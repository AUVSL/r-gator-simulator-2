#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the gazebo_ros package launch files
    gazebo_ros_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
    ])
    
    # Path to the robot's URDF file
    rgator_model_urdf_path = PathJoinSubstitution([
        FindPackageShare('r-gator-model-2'), 'urdf', 'rgator_model.urdf'
    ])

    # Define the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Include the empty_world launch file from gazebo_ros
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_ros_launch_path),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Node to publish static transform between base_link and base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # Node to spawn the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=['-file', rgator_model_urdf_path, '-entity', 'rgator_model'],
            output='screen'
        ),

        # Node to publish that the robot is calibrated (using a boolean message)
        Node(
            package='topic_tools',
            executable='relay',
            name='fake_joint_calibration',
            arguments=['/calibrated', '/calibrated'],
            remappings=[('/calibrated', 'std_msgs/msg/Bool')],
            parameters=[{'data': True}]
        ),
    ])
