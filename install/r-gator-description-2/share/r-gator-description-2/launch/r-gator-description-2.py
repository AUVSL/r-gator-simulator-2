#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='/')

    return LaunchDescription([
        DeclareLaunchArgumetf2_geometry_msgsnt(
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
                    namespace=namespace,
                    parameters=[{
                        'robot_description': Command([
                            FindExecutable(name='xacro'),
                            ' ',
                            PathJoinSubstitution([
                                FindPackageShare('rgator_model'),
                                'urdf',
                                'rgator_model.urdf.xacro'
                            ])
                        ]),
                        'publish_frequency': 30.0,
                    }],
                ),
            ],
            # LaunchConfiguration is used to fetch the 'namespace' argument value
            namespace=namespace,
        ),
    ])
