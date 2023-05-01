import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'robot.launch.py')))
    
    interface_node = Node(
        package="my_tb3_launcher",
        executable="nav2Interface.py"
    )
    
    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    ld.add_action(interface_node)
    return ld