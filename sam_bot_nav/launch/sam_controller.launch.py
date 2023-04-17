import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get the launch directory
    robot_name = LaunchConfiguration("robot_name")
    x_pos = LaunchConfiguration("x_pos")
    y_pos = LaunchConfiguration("y_pos")
    yaw = LaunchConfiguration("yaw")
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value="")
    declare_x_pos = DeclareLaunchArgument('x_pos', default_value="0.0")
    declare_y_pos = DeclareLaunchArgument('y_pos', default_value="0.0")
    declare_yaw = DeclareLaunchArgument('yaw', default_value="0.0")
    robot_nav_controller = Node(
        package="sam_bot_nav",
        executable="navigatorBridge.py",
        parameters=[
            {"robot_name": "robot_1"},
            {"x_pos": x_pos},
            {"y_pos": y_pos},
            {"yaw": yaw}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_robot_name)
    ld.add_action(declare_x_pos)
    ld.add_action(declare_y_pos)
    ld.add_action(declare_yaw)
    ld.add_action(robot_nav_controller)

    return ld