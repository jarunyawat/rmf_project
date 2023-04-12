import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.1,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.1,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}]
    nav_instances_cmds = []
    for robot in robots:
        robot_state_publisher_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot['name'],
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        )

        rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            namespace=robot['name'],
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        )

        spawn_entity = launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace=robot['name'],
            arguments=['-entity', robot['name'],
                    '-topic', 'robot_description',
                    "-robot_namespace", robot['name'],
                    '-x', str(robot['x_pose']),
                    '-y', str(robot['y_pose']),
                    '-z', str(robot['z_pose']),
                    '-R', str(robot['roll']),
                    '-P', str(robot['pitch']),
                    '-Y', str(robot['yaw'])],
            output='screen'
        )

        robot_localization_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            namespace=robot['name'],
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
        group = launch.actions.GroupAction([
            robot_state_publisher_node,
            rviz_node,
            spawn_entity,
            robot_localization_node,
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = launch.LaunchDescription()
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'))
    ld.add_action(launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'))
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)
    return ld