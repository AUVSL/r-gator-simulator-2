from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch arguments
    paused_arg = DeclareLaunchArgument('paused', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='highbay_track.world')
    x_arg = DeclareLaunchArgument('x', default_value='-1.5')
    y_arg = DeclareLaunchArgument('y', default_value='-21')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'verbose': 'false',
            'world_name': PathJoinSubstitution([
                FindPackageShare('r_gator_gazebo'),
                'worlds',
                LaunchConfiguration('world_name')
            ])
        }.items()
    )

    # Include the r_gator_vehicle launch file
    r_gator_vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('r_gator_gazebo'),
                'launch',
                'r_gator_vehicle.launch.py'
            ])
        ]),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw')
        }.items()
    )

    # Node to launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('r_gator_launch'),
            'config_rviz',
            'r_gator_velodyne.rviz'
        ])],
    )

    return LaunchDescription([
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        world_name_arg,
        x_arg,
        y_arg,
        yaw_arg,
        gazebo_launch,
        r_gator_vehicle_launch,
        rviz_node
    ])
