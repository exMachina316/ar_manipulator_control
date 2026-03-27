import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

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
    extrincs_calibration = LaunchConfiguration('extrinsics_calibration', default='False')
    use_waypoint_manager = LaunchConfiguration("waypoint_manager", default='False')

    camera_params_file = os.path.join(follow_traj_pkg_share, "config", "multi_camera.yaml")

    # cams = ["oak1"]
    cams = ["oak1", "oak3"]
    # cams = ["oak1", "oak2", "oak3"]

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
                "use_waypoint_manager": use_waypoint_manager,
            }.items(),
        )

        print(f"Setting up camera: {cam_name} for TF transforms to the EEF marker")
        extrinsics_file = os.path.join(follow_traj_pkg_share, "config", "calibration", f"{cam_name}_extrinsics.yaml")
        tf_args = get_static_tf_args(extrinsics_file)
        
        if tf_args: # Only create the node if arguments were found
            tf_publisher_node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_publisher',
                condition=UnlessCondition(extrincs_calibration),
                namespace=cam_name,
                arguments=tf_args,
                output='screen',
                parameters=[{"use_sim_time": use_sim_time}],
            )
            nodes.append(tf_publisher_node)
        nodes.append(node)

    tf_inverter_node = Node(
        package='follow_traj_moveit',
        executable='tf_inverter_node',
        name='tf_inverter_node',
        condition=IfCondition(extrincs_calibration),
        output='screen',
        parameters=[{'camera_names': cams, 'use_sim_time': use_sim_time}]
    )

    eff_tf_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='eff_tf_broadcaster_node',
        condition=IfCondition(extrincs_calibration),
        output='screen',
        arguments=['0', '0', '0.01', '3.1414', '0', '0', 'tool0', 'eef_marker']
    )

    extrincs_calibration_node = Node(
        package='follow_traj_moveit',
        executable='extrinsics_calibrator',
        name='extrinsics_calibrator',
        condition=IfCondition(extrincs_calibration),
        output='screen',
        parameters=[{'camera_names': cams, 'use_sim_time': use_sim_time}]
    )

    calibration_nodes = [
        tf_inverter_node,
        eff_tf_broadcaster_node,
        extrincs_calibration_node,
    ]

    return nodes + calibration_nodes


def get_static_tf_args(extrinsics_file):
    # This function loads the YAML and extracts the transform values
    with open(extrinsics_file, 'r') as f:
        config = yaml.safe_load(f)
    
    t = config.get('translation', {})
    r = config.get('rotation', {})
    
    frame_id = config.get('frame_id')
    child_frame_id = config.get('child_frame_id')

    if not all([frame_id, child_frame_id, t, r]):
        # Handle case where params are missing
        return None

    # static_transform_publisher quaternion argument order:
    # x y z qx qy qz qw frame_id child_frame_id
    args = [
        str(t['x']), str(t['y']), str(t['z']), 
        str(r['x']), str(r['y']), str(r['z']), str(r['w']), 
        frame_id, child_frame_id
    ]
    return args


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('extrinsics_calibration', default_value='False'),
        DeclareLaunchArgument("waypoint_manager", default_value="False"),
    ]
        
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )