from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    telemetrix=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mirte_telemetrix_cpp'),
                'launch',
                'telemetrix.launch.py'
            ])
        ])
    )


#     diff_drive_control=IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             PathJoinSubstitution([
#                 FindPackageShare('mirte_control'),
#                 'launch',
#                 'diffbot.launch.py'
#             ])
#         ])
#     )

    mecanum_drive_control=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mirte_base_control'),
                "bringup",'launch',
                'mirte_base.launch.py'
            ])
        ])
    )

    ld.add_action(telemetrix)
    ld.add_action(mecanum_drive_control)
    return ld
