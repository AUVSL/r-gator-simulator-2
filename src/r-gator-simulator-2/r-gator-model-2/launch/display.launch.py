#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model_arg = DeclareLaunchArgument('model', default_value='default_model')
    model = LaunchConfiguration('model')
    rgator_model_path = PathJoinSubstitution([
        FindPackageShare('rgator_model'), 'urdf', 'rgator_model.urdf'
    ])
    rgator_rviz_config_path = PathJoinSubstitution([
        FindPackageShare('rgator_model'), 'urdf.rviz'
    ])

    return LaunchDescription([
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', rgator_model_path])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rgator_rviz_config_path]
        ),
    ])
