import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    sam_bot_desc_dir = get_package_share_directory('sam_bot_description')
    sam_bot_nav_dir = get_package_share_directory('sam_bot_nav')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    default_model_path = os.path.join(sam_bot_desc_dir, 'src/description/sam_bot_description.urdf')

    # Create the launch configuration variables
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    
    # file directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.1,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'param': LaunchConfiguration(f'robot1_params_file'), 'ekf': "ekf_robot1.yaml"},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.1,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'param': LaunchConfiguration(f'robot2_params_file'), 'ekf': "ekf_robot2.yaml"}]

    # robots = [
    #     {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.1,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'param': LaunchConfiguration(f'robot1_params_file')}]
    
    # declare launch argument
    # Declare the launch arguments
    declare_model = DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sam_bot_nav_dir, 'world', 'office_world.sdf'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(sam_bot_nav_dir, 'map', 'office_map.yaml'),
        description='Full path to map file to load')

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        'robot1_params_file',
        default_value=os.path.join(sam_bot_nav_dir, 'config', 'sam_bot_1_nav_param.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        'robot2_params_file',
        default_value=os.path.join(sam_bot_nav_dir, 'config', 'sam_bot_2_nav_param.yaml'),
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')
    
    nav_instances_cmds = []
    for robot in robots:
        robot_state_publisher_node = launch_ros.actions.Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot['name'],
            parameters=[{'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])}],
            remappings=[('/tf', f"/{robot['name']}/tf"),
                  ('/tf_static', f"/{robot['name']}/tf_static")]
        )

        rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': robot['name'],
                          'use_namespace': TextSubstitution(text=str("True")),
                          'rviz_config': rviz_config_file}.items())

        bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': robot['name'],
                          'use_namespace': TextSubstitution(text=str("True")),
                          'slam': TextSubstitution(text=str("False")),
                          'map': map_yaml_file,
                          'params_file': robot['param'],
                          'autostart': autostart,
                          'use_sim_time': use_sim_time,
                          'use_composition': TextSubstitution(text=str("False")),
                          'use_respawn': use_respawn}.items())

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

        robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            namespace=robot['name'],
            parameters=[os.path.join(sam_bot_nav_dir, f"config/{robot['ekf']}"), {'use_sim_time': use_sim_time}],
            remappings=[('/tf', f"/{robot['name']}/tf"),
                  ('/tf_static', f"/{robot['name']}/tf_static")]
        )
        group = launch.actions.GroupAction([
            robot_state_publisher_node,
            rviz_cmd,
            spawn_entity,
            robot_localization_node,
            bringup_cmd,
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_model)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(start_gazebo_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)
    return ld