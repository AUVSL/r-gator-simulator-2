import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_dir =get_package_share_directory('<package_path>') # add package path
    world_files = os.path.join(package_dir,'worlds','<world_path>') # add world file path for *.world file

    ld = LaunchDescription()

    gazeboworld = ExecuteProcess(
        cmd=['gazebo','--verbose',world_files,'-s','libgazebo_ros_factory.so']
    )

    ld.add_action(gazeboworld)
    return ld

def generate_launch_description():

urdf = os.path.join(get_package_share_directory(
        robot_name), 'urdf', '<File>.urdf')

xml = open(urdf, 'r').read()

xml = xml.replace('"', '\\"')

swpan_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'

return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen'),
    ])