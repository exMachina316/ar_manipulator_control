import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace").perform(context)
    name = LaunchConfiguration("name").perform(context)
    rectify_rgb = LaunchConfiguration("rectify_rgb")
    use_apriltag = LaunchConfiguration("use_apriltag")

    return [
        ComposableNodeContainer(
            name=f"camera_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    namespace=namespace,
                    parameters=[
                        params_file,
                    ],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
        LoadComposableNodes(
            condition=IfCondition(rectify_rgb),
            target_container=f"{namespace}/camera_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    namespace=namespace,
                    parameters=[params_file],
                    remappings=[
                        ("image", f"{name}/rgb/image_raw"),
                        ("camera_info", f"{name}/rgb/camera_info"),
                        ("image_rect", f"{name}/rgb/image_rect"),
                        (
                            "image_rect/compressed",
                            f"{name}/rgb/image_rect/compressed",
                        ),
                        (
                            "image_rect/compressedDepth",
                            f"{name}/rgb/image_rect/compressedDepth",
                        ),
                        (
                            "image_rect/theora",
                            f"{name}/rgb/image_rect/theora",
                        ),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(use_apriltag),
            target_container=f"{namespace}/camera_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="apriltag_ros",
                    plugin="AprilTagNode",
                    name="apriltag",
                    namespace=namespace,
                    parameters=[params_file],
                    remappings=[
                        ("image_rect", f"{name}/rgb/image_rect"),
                        ("camera_info", f"{name}/rgb/camera_info"),
                    ],
                )
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera.yaml"),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="true"),
        DeclareLaunchArgument("use_apriltag", default_value="true"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )