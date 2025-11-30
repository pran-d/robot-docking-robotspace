import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_bot'

    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'launch_sim.launch.py')])
    )

    aruco_detector = Node(
        package='my_bot',
        executable='aruco_detector',
        output='screen'
    )

    docking_node = Node(
        package='my_bot',
        executable='docking_node',
        output='screen'
    )

    return LaunchDescription([
        launch_sim,
        aruco_detector,
        docking_node,
    ])
