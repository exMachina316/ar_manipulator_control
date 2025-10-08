import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    follow_traj_pkg_share = get_package_share_directory("follow_traj_moveit")
    params_file = os.path.join(follow_traj_pkg_share, "config", "multi_camera.yaml")
    cams = ["oak1", "oak2", "oak3"]

    nodes = []
    for cam_name in cams:
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(follow_traj_pkg_share, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": cam_name,
                "namespace": cam_name,
                "params_file": params_file,
            }.items(),
        )
        nodes.append(node)

    return nodes


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )