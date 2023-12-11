#!/usr/bin/env python3
import launch_ros.descriptions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='/')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace'
        ),

        GroupAction(
            actions=[
                # The robot_description parameter holds the URDF description of the robot
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace=namespace,  # Correct place for the namespace
                    parameters=[{
                        'robot_description': ParameterValue(Command([
                            FindExecutable(name='xacro'),
                            ' ',
                            PathJoinSubstitution([
                                FindPackageShare('r-gator-model-2'),
                                'urdf',
                                'rgator_model.urdf.xacro'
                            ])
                        ]), value_type=str),
                        'publish_frequency': 30.0,
                    }],
                ),
            ]
        ),
    ])
