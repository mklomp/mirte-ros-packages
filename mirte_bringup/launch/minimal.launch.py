import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "machine_namespace",
                # default_value=TextSubstitution(text=platform.node().replace("-", "_")),
                default_value="mirte"
            )
        ]
    )

    # ld.add_action(PushRosNamespace(LaunchConfiguration("machine_namespace")))

    telemetrix = GroupAction([PushRosNamespace(LaunchConfiguration("machine_namespace")),IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("mirte_telemetrix_cpp"),
                        "launch",
                        "telemetrix.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config_path": PathJoinSubstitution(
                [
                    FindPackageShare("mirte_telemetrix_cpp"),
                    "config",
                    "mirte_user_config.yaml",
                ]
            )
        }.items(),
    )])

    diff_drive_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("mirte_control"), "launch", "diffbot.launch.py"]
                )
            ]
        )
    )

    ld.add_action(telemetrix)
    ld.add_action(diff_drive_control)
    return ld
