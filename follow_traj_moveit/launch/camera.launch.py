import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    mr_manip_share = FindPackageShare('mr_manipulator')

    # Launch configuration variables
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace").perform(context)
    name = LaunchConfiguration("name").perform(context)
    rectify_rgb = LaunchConfiguration("rectify_rgb")
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    use_waypoint_manager = LaunchConfiguration("waypoint_manager", default='False')

    model_path = PathJoinSubstitution([mr_manip_share, 'models', 'xgboost_model.1.0.0.p'])

    camera_node_container = ComposableNodeContainer(
        name=f"camera_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name=name,
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    load_rectify_node = LoadComposableNodes(
        condition=IfCondition(rectify_rgb),
        target_container=f"{namespace}/camera_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_color_node",
                namespace=namespace,
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ("image", "rgb/image_raw"),
                    ("camera_info", "rgb/camera_info"),
                    ("image_rect", "rgb/image_rect"),
                    (
                        "image_rect/compressed",
                        "rgb/image_rect/compressed",
                    ),
                    (
                        "image_rect/compressedDepth",
                        "rgb/image_rect/compressedDepth",
                    ),
                    (
                        "image_rect/theora",
                        "rgb/image_rect/theora",
                    ),
                ],
            )
        ],
    )

    load_apriltag_node = LoadComposableNodes(
        condition=IfCondition(use_apriltag),
        target_container=f"{namespace}/camera_container",
        composable_node_descriptions=[
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace=namespace,
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ("image_rect", "rgb/image_rect"),
                    ("camera_info", "rgb/camera_info"),
                ],
            )
        ],
    )

    camera_layer = [camera_node_container, load_rectify_node, load_apriltag_node]

    hand_drawing_node = Node(
        package='mr_manipulator',
        executable='hand_drawing',
        name='hand_drawing',
        namespace=namespace,
        output='screen',
        condition=IfCondition(use_waypoint_manager),
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
        }],
        remappings=[
            ('/camera/image', 'rgb/image_rect'),
            ('/camera/camera_info', 'rgb/camera_info'),
        ]
    )

    perception_layer = [
        hand_drawing_node,
    ]

    return camera_layer + perception_layer


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("namespace", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera.yaml"),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="true"),
        DeclareLaunchArgument("use_apriltag", default_value="true"),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument("waypoint_manager", default_value="False"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )