from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    dispenser = Node(
        package='sam_rmf',
        executable='dispenser_mock.py',
        name='tool_dispensor_1_node',
        parameters=[{"handle_name": "tool_dispensor_1"}])
    
    ingestor_1 = Node(
        package='sam_rmf',
        executable='ingestor_mock.py',
        name='tool_ingestor_1_node',
        parameters=[{"handle_name": "tool_ingestor_1"}]
    )

    ingestor_2 = Node(
        package='sam_rmf',
        executable='ingestor_mock.py',
        name='tool_ingestor_2_node',
        parameters=[{"handle_name": "tool_ingestor_2"}]
    )

    ingestor_3 = Node(
        package='sam_rmf',
        executable='ingestor_mock.py',
        name='tool_ingestor_3_node',
        parameters=[{"handle_name": "tool_ingestor_3"}]
    )
    ld = LaunchDescription()
    ld.add_action(dispenser)
    ld.add_action(ingestor_1)
    ld.add_action(ingestor_2)
    ld.add_action(ingestor_3)
    return ld