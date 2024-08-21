from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    mecanum_drive_control=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mirte_base_control'),
                'launch',
                'mirte_base.launch.py'
            ])
        ])
    )
    
    web_video_server=Node(
        package = 'web_video_server',
        executable = 'web_video_server', 
        output='screen'
    )

    depth_cam = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astro_pro_plus.launch.xml'
            ])
        ])
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            ])
        ])
    )

    ld.add_action(telemetrix)
    ld.add_action(mecanum_drive_control)
    ld.add_action(web_video_server)
    ld.add_action(lidar)
    ld.add_action(depth_cam)
    return ld
