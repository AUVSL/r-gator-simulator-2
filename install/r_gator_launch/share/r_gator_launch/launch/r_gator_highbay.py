from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value="false"),
        DeclareLaunchArgument('use_sim_time', default_value="true"),
        DeclareLaunchArgument('gui', default_value="true"),
        DeclareLaunchArgument('headless', default_value="false"),
        DeclareLaunchArgument('debug', default_value="false"),
        DeclareLaunchArgument('world_name', default_value="highbay_track.world"),
        DeclareLaunchArgument('vehicle_x', default_value="-10"),
        DeclareLaunchArgument('vehicle_y', default_value="-21"),
        DeclareLaunchArgument('vehicle_yaw', default_value="3.14"),
        DeclareLaunchArgument('configuration_basename', default_value="r_gator_slam_2d_gazebo.lua"),
        DeclareLaunchArgument('remap_map_topic', default_value="false"),
        DeclareLaunchArgument('remap_map_topic_name', default_value="/cmap"),
        
        LogInfo(condition=UnlessCondition(LaunchConfiguration('headless')),
                namespace='Launch',
                message=["Gazebo Arguments: --debug", LaunchConfiguration('debug'), "--gui", LaunchConfiguration('gui'), "--paused", LaunchConfiguration('paused'), "--use_sim_time", LaunchConfiguration('use_sim_time'), "-s libgazebo_ros_factory.so", os.path.join(get_package_share_directory('r_gator_gazebo'), 'worlds', LaunchConfiguration('world_name'))]),
        
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=[
                '--debug', LaunchConfiguration('debug'),
                '--gui', LaunchConfiguration('gui'),
                '--paused', LaunchConfiguration('paused'),
                '--use_sim_time', LaunchConfiguration('use_sim_time'),
                '-s', 'libgazebo_ros_factory.so',
                os.path.join(get_package_share_directory('r_gator_gazebo'), 'worlds', LaunchConfiguration('world_name'))
            ],
            condition=UnlessCondition(LaunchConfiguration('headless'))
        ),
        # Include other nodes as needed
    ])
