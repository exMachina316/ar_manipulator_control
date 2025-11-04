import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    follow_traj_pkg_share = get_package_share_directory("follow_traj_moveit")

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    camera_params_file = os.path.join(follow_traj_pkg_share, "config", "multi_camera.yaml")

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
                "params_file": camera_params_file,
                "use_sim_time": use_sim_time,
            }.items(),
        )
        nodes.append(node)

    tf_inverter_node = Node(
        package='follow_traj_moveit',
        executable='tf_inverter_node',
        name='tf_inverter_node',
        output='screen',
        parameters=[{'camera_names': cams, 'use_sim_time': use_sim_time}]
    )
    nodes.append(tf_inverter_node)

    return nodes


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='False'),
    ]
        
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )