from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    param_config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution((
            get_package_share_directory("mirte_telemetrix_cpp"),
            "config",
            "mirte_user_config.yaml",
        )),
    )
    ld = LaunchDescription([param_config_arg])

    # config = os.path.join(
    # get_package_share_directory("mirte_telemetrix_cpp"),
    # "config",
    #     "mirte_user_config.yaml",
    # )

    node = Node(
        package="mirte_telemetrix_cpp",
        name="mirte_telemetrix",
        executable="mirte_telemetrix_cpp_node",
        parameters=[LaunchConfiguration("config_path")],
        output="screen",
        namespace="/mirte",
        # node_namespace='/mirte',
        emulate_tty=True,
    )

    ld.add_action(node)
    return ld
