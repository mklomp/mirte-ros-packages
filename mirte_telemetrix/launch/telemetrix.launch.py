from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    param_config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution(
            (
                get_package_share_directory("mirte_telemetrix"),
                "config",
                "mirte_user_config.yaml",
            )
        ),
    )

    ld = LaunchDescription([param_config_arg])

    node = Node(
        package="mirte_telemetrix",
        name="mirte_telemetrix",
        executable="mirte_telemetrix",
        parameters=[LaunchConfiguration("config_path")],
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(node)
    return ld
