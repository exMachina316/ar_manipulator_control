import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    follow_traj_moveit_share_dir = os.path.join(
        get_package_share_directory('follow_traj_moveit')
    )

    # Include system_bringup.launch.py
    system_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_traj_moveit_share_dir, 'launch', 'system_bringup.launch.py')
        )
    )

    # Include multi_camera.launch.py
    multi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follow_traj_moveit_share_dir, 'launch', 'multi_camera.launch.py')
        )
    )

    return LaunchDescription([
        system_bringup_launch,
        multi_camera_launch
    ])