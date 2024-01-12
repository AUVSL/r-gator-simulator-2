#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the YAML file using PathJoinSubstitution
    config_file_path = PathJoinSubstitution([
        FindPackageShare('r-gator-gazebo-2'), 'config', 'r_gator_ackermann_control_params.yaml'
    ])
  
    # Launch arguments
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='highbay_track.world')
    vehicle_x_arg = DeclareLaunchArgument('vehicle_x', default_value='-10')
    vehicle_y_arg = DeclareLaunchArgument('vehicle_y', default_value='-21')
    vehicle_yaw_arg = DeclareLaunchArgument('vehicle_yaw', default_value='3.14')


    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('r-gator-model-2'),
                'launch',
                'display.launch.py'
            ])
        ])
    )

    gazebo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'world': PathJoinSubstitution([
                FindPackageShare('r-gator-gazebo-2'),
                'worlds',
                LaunchConfiguration('world_name')
            ]),
        }.items()
    )

    cmd_to_ackermann_node = Node(
        package='r-gator-gazebo-tool-2',
        executable='cmd-vel-to-ackermann-drive.py',
        name='cmd_vel_to_ackermann_drive',
        output='screen'
    )

    # Define the GEM Ackermann Controller node
    gem_ackermann_controller_node = Node(
        package='r-gator-gazebo-2',  # Ensure this is the correct package
        executable='r_gator_control.py',  # Ensure this is the correct executable
        name='gem_controller',
        output='screen',
        parameters=[config_file_path]  # Ensure the YAML configuration is correct
    )
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'rgator_model', 
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('vehicle_x'),
                    '-y', LaunchConfiguration('vehicle_y'),  
                    '-z', '0.5',
                    '-Y', LaunchConfiguration('vehicle_yaw')],
        output='screen'
    )
    controllers = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_file_path]  # Load the controller configurations
    )
    # Environment variable to set use_sim_time for all nodes launched in this file
    set_use_sim_time = SetEnvironmentVariable('use_sim_time', LaunchConfiguration('use_sim_time'))

    return LaunchDescription([
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        world_name_arg,
        vehicle_x_arg,
        vehicle_y_arg,
        vehicle_yaw_arg,
        description_launch,
        gazebo_ros_launch,
        controllers,
        cmd_to_ackermann_node,
        set_use_sim_time,
        spawn_entity_cmd,
        gem_ackermann_controller_node
          # Ensure this is at the end to load parameters last
    ])
