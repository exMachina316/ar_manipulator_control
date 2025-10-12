from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mr_manip_share = FindPackageShare('mr_manipulator')
    model_path = PathJoinSubstitution([mr_manip_share, 'models', 'xgboost_model.p'])
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    planning_frame = LaunchConfiguration('planning_frame', default='world')

    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock or system clock'
    )

    declare_target_frame_cmd = DeclareLaunchArgument(
        'planning_frame',
        default_value='world',
        description='Target frame for trajectory transformation'
    )

    # Nodes
    test_waypoint_pub_node = Node(
        package='mr_manipulator',
        executable='test_waypoint_pub',
        name='test_waypoint_pub',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    action_client_node = Node(
        package='mr_manipulator',
        executable='action_client',
        name='action_client',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    action_server_node = Node(
        package='follow_traj_moveit',
        executable='action_server',
        name='cartesian_path_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    trajectory_preprocessor_node = Node(
        package='follow_traj_moveit',
        executable='traj_preprocess',
        name='trajectory_preprocessor',
        output='screen',
        parameters=[{
            'planning_frame': planning_frame,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_target_frame_cmd,
        
        # Nodes
        action_server_node,
        action_client_node,
        trajectory_preprocessor_node
    ])