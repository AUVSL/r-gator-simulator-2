#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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

    # Convert include tags to IncludeLaunchDescription
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

    # Nodes definition
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen'
    )

    cmd_to_ackermann_node = Node(
        package='r-gator-gazebo-tool-2',
        executable='cmd-vel-to-ackermann-drive.py',
        name='cmd_vel_to_ackermann_drive',
        output='screen'
    )

    r_gator_teleop_node = Node(
        package='r-gator-teleop-2',
        executable='r_gator_teleop_node',
        name='r_gator_teleop_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('r-gator-launch-2'),
            'config_rviz',
            'r-gator-velodyne.rviz'
        ])],
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
        gazebo_ros_launch,
        teleop_node,
        cmd_to_ackermann_node,
        r_gator_teleop_node,
        rviz_node,
        set_use_sim_time
    ])
