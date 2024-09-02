import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
import yaml
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mirte_telemetrix_cpp'),
        'config',
        'mirte_user_config.yaml'
        )
    param_config = os.path.join(
        get_package_share_directory('mirte_telemetrix_cpp'),
        'config',
        'mirte_user_config.yaml'
        )
    
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)
    print(params)
    # container = ComposableNodeContainer(
    #         package = 'mirte_telemetrix_cpp',
    #         name = 'mirte_telemetrix',
    #         executable = 'mirte_telemetrix_cpp_node',
    #         node_namespace='/mirte',
    #         composable_node_descriptions=[
    #             ComposableNode(
    #                 node_name='ros2_to_serial_bridge',
    #                 node_namespace='/mirte',
    #                 package='ros2_serial_example',
    #                 node_plugin='ros2_to_serial_bridge::ROS2ToSerialBridge',
    #                 parameters=[params]),
    #         ],
    #         output='screen',
    # )

    node=Node(
        package = 'mirte_telemetrix_cpp',
        name = 'mirte_telemetrix',
        executable = 'mirte_telemetrix_cpp_node',
        parameters = [config],
        output='screen',
        namespace='/mirte',
        # node_namespace='/mirte',
        emulate_tty=True
    )

    ld.add_action(node)
    return ld