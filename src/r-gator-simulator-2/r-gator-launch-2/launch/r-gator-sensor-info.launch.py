from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gem_gazebo',
            executable='r_gator_sensor_info.py',
            name='gem_sensor_info'
        ),
    ])
