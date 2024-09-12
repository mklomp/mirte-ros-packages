from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution(
                (
                    FindPackageShare("mirte_telemetrix_cpp"),
                    "config",
                    "mirte_user_config.yaml",
                )
            ),
        ),
        DeclareLaunchArgument(
            "hardware_namespace",
            default_value="io",
            description="The namespace for the Telemetrix Node and the hardware peripherals",
        ),
    ]

    ld = LaunchDescription(launch_arguments)

    node = Node(
        package="mirte_telemetrix_cpp",
        name="telemetrix",
        executable="mirte_telemetrix_cpp_node",
        parameters=[LaunchConfiguration("config_path")],
        output="screen",
        emulate_tty=True,
        namespace=LaunchConfiguration("hardware_namespace"),
        respawn=True,
        respawn_delay=5,
        # TODO: Not avialable yet in humble (avialable starting from jazzy)
        # respawn_max_retries=10
    )

    ld.add_action(node)
    return ld
