from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    home_lane_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sam_rmf'),
                    "launch",
                    'home.launch.xml'
                ])
            ])
    )

    station_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sam_rmf'),
                    "launch",
                    'station.launch.py'
                ])
            ])
    )
    ld = LaunchDescription()
    ld.add_action(home_lane_launch)
    ld.add_action(station_launch)
    return ld