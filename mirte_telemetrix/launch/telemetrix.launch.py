import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mirte_telemetrix'),
        'config',
        'mirte_user_config.yaml'
        )
        
    node=Node(
        package = 'mirte_telemetrix',
        name = 'mirte_telemetrix',
        executable = 'mirte_telemetrix',
        parameters = [config],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(node)
    return ld
