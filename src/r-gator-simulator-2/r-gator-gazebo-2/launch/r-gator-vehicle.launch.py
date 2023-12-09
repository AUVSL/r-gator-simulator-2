#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='/')
    cmd_timeout_arg = DeclareLaunchArgument('cmd_timeout', default_value='0.5')
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='-98.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.3')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')

    # Include the r_gator_description launch file
    r_gator_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('r_gator_description'),
                'launch',
                'r_gator_description.launch.py'
            ])
        ]),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items()
    )

    # Group to namespace the nodes
    namespaced_group = GroupAction(
        actions=[
            # Node to spawn the model in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_model',
                arguments=[
                    '-entity', 'r_gator',
                    '-topic', 'robot_description',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z'),
                    '-Y', LaunchConfiguration('yaw'),
                    '-P', LaunchConfiguration('pitch'),
                    '-R', LaunchConfiguration('roll')
                ]
            ),

            # Node to load and start controllers
            Node(
                package='controller_manager',
                executable='spawner.py',
                name='controller_spawner',
                arguments=[
                    'r_gator_joint_control_params',  # Assuming the controllers' config is converted to ROS2 format
                    '--param-file', PathJoinSubstitution([
                        FindPackageShare('r_gator_gazebo'),
                        'config',
                        'r_gator_joint_control_params.yaml'
                    ])
                ]
            ),

            # Node for the custom r_gator control
            Node(
                package='r_gator_gazebo',
                executable='r_gator_control.py',
                name='ackermann_controller',
                parameters=[
                    {'cmd_timeout': LaunchConfiguration('cmd_timeout')},
                    PathJoinSubstitution([
                        FindPackageShare('r_gator_gazebo'),
                        'config',
                        'r_gator_ackermann_control_params.yaml'
                    ])
                ]
            ),

            # Node for publishing joint states
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[
                    {'rate': 10},  # The rate is now set as a parameter
                    {'use_gui': False}
                ]
            )
        ],
        # Namespace configuration
        scoped_launch_configurations=[
            SetLaunchConfiguration('namespace', LaunchConfiguration('namespace'))
        ]
    )

    return LaunchDescription([
        namespace_arg,
        cmd_timeout_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        verbose_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        roll_arg,
        pitch_arg,
        r_gator_description_launch,
        namespaced_group
    ])
