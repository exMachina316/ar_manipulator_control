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

    parameter_overrides = {
        "camera": {
            "i_enable_imu": False,
            "i_pipeline_type": "RGB",
            "i_nn_type": "RGB",
        }
    }
    color_sens_name = "rgb"

    return [
        ComposableNodeContainer(
            name=f"{name}_container",
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
                        parameter_overrides,
                    ],
                )
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        ),
        LoadComposableNodes(
            condition=IfCondition(rectify_rgb),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    namespace=namespace,
                    parameters=[params_file],
                    remappings=[
                        ("image", f"{name}/{color_sens_name}/image_raw"),
                        ("camera_info", f"{name}/{color_sens_name}/camera_info"),
                        ("image_rect", f"{name}/{color_sens_name}/image_rect"),
                        (
                            "image_rect/compressed",
                            f"{name}/{color_sens_name}/image_rect/compressed",
                        ),
                        (
                            "image_rect/compressedDepth",
                            f"{name}/{color_sens_name}/image_rect/compressedDepth",
                        ),
                        (
                            "image_rect/theora",
                            f"{name}/{color_sens_name}/image_rect/theora",
                        ),
                    ],
                )
            ],
        ),
        LoadComposableNodes(
            condition=IfCondition(use_apriltag),
            target_container=f"{namespace}/{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="apriltag_ros",
                    plugin="apriltag_ros::AprilTagNode",
                    name="apriltag",
                    namespace=namespace,
                    parameters=[params_file],
                    remappings=[
                        ("image_rect", f"{name}/{color_sens_name}/image_rect"),
                        ("camera_info", f"{name}/{color_sens_name}/camera_info"),
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