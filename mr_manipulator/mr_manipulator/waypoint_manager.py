from enum import Enum
from functools import partial

from collections import Counter

import numpy as np

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose, PointStamped

from ur_interfaces.action import ExecuteWayPointsWithTypes

class OperationMode(Enum):
    WAYPOINT_REGISTRATION = 1
    TRAJECTORY_TYPE_SELECTION = 2


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('camera_names', ['camera'])
        self.declare_parameter('world_frame', 'world')

        self.camera_names = self.get_parameter('camera_names').get_parameter_value().string_array_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = rclpy.qos.qos_profile_sensor_data

        # Create ROS 2 publishers
        self.status_marker_publisher = self.create_publisher(Marker, '/status_marker', sensor_qos)

        # Publishers
        self.waypoints_publisher = self.create_publisher(PoseArray, '/waypoints', 10)
        self.triag_finger_tip_pub = self.create_publisher(PointStamped, '/triangulated_finger_tip', 10)

        # Action Client for trajectory execution
        self._action_client = ActionClient(self, ExecuteWayPointsWithTypes, 'plan_execute_pilz_path')
        self.executing = False

        # State and subscribers per camera
        self.camera_states = {}
        for name in self.camera_names:
            self.camera_states[name] = {
                'left_gesture': "",
                'right_gesture': "",
                'finger_tip_pose': None,
                'subs': []
            }

            left_gesture_sub = self.create_subscription(
                String, f'/{name}/left_hand_gesture', partial(self.left_gesture_callback, camera_name=name), 10)
            right_gesture_sub = self.create_subscription(
                String, f'/{name}/right_hand_gesture', partial(self.right_gesture_callback, camera_name=name), 10)
            # finger tip comes in as a Vector3Stamped (vector in camera frame)
            finger_tip_sub = self.create_subscription(
                PointStamped, f'/{name}/finger_tip_pose', partial(self.finger_tip_callback, camera_name=name), 10)

            self.camera_states[name]['subs'].extend([left_gesture_sub, right_gesture_sub, finger_tip_sub])
            self.get_logger().info(f"Subscribed to topics for camera: {name}")

        # Waypoints and trajectory information
        self.waypoints = []
        self.trajectory_types = []  # Types between consecutive waypoints
        self.interim_points = []  # Interim points for CIRC trajectories
        self.interim_point_indices = []  # Which segments have interim points

        # Operation mode
        self.current_mode = OperationMode.WAYPOINT_REGISTRATION
        self.get_logger().info(f"Starting in mode: {self.current_mode.name}")

        # Global gesture tracking states
        self.hold_start_time = None
        self.hold_triggered = False

        # Trajectory type selection gesture states
        self.right_peace_start_time = None
        self.right_peace_triggered = False
        self.left_peace_start_time = None
        self.left_peace_triggered = False
        self.right_hold_start_time = None
        self.right_hold_triggered = False

        # Clear waypoints gesture (left peace in waypoint mode)
        self.clear_peace_start_time = None
        self.clear_peace_triggered = False

        # Execute gesture (right peace in waypoint mode)
        self.execute_peace_start_time = None
        self.execute_triggered = False

        # State for waiting for interim point after CIRC selection
        self.waiting_for_interim_point = False
        self.pending_trajectory_type = None

        self.timer = self.create_timer(0.1, self.process_gestures)

    def left_gesture_callback(self, msg, camera_name):
        self.camera_states[camera_name]['left_gesture'] = msg.data

    def right_gesture_callback(self, msg, camera_name):
        self.camera_states[camera_name]['right_gesture'] = msg.data

    def finger_tip_callback(self, msg, camera_name):
        self.camera_states[camera_name]['finger_tip_pose'] = msg

    def get_dominant_gesture(self, hand):
        gestures = [self.camera_states[name][f'{hand}_gesture'] for name in self.camera_names if self.camera_states[name][f'{hand}_gesture']]
        if not gestures:
            return None
        # Simple majority vote for now.
        # A softmax approach would require probability scores from the gesture model.
        return Counter(gestures).most_common(1)[0][0]

    def switch_mode(self):
        """Toggle between WAYPOINT_REGISTRATION and TRAJECTORY_TYPE_SELECTION modes"""
        if self.current_mode == OperationMode.WAYPOINT_REGISTRATION:
            self.current_mode = OperationMode.TRAJECTORY_TYPE_SELECTION
        else:
            self.current_mode = OperationMode.WAYPOINT_REGISTRATION

        self.get_logger().info(f"Switched to mode: {self.current_mode.name}")

    def process_gestures(self):
        dominant_left_gesture = self.get_dominant_gesture('left')
        dominant_right_gesture = self.get_dominant_gesture('right')

        triangulated_pose = self._trinangulate_fingertip_pose()

        if dominant_left_gesture == 'Pointer':
            triangulated_point = PointStamped()
            triangulated_point.header.stamp = self.get_clock().now().to_msg()
            triangulated_point.header.frame_id = self.world_frame

            triangulated_point.point.x = triangulated_pose.position.x
            triangulated_point.point.y = triangulated_pose.position.y
            triangulated_point.point.z = triangulated_pose.position.z

            self.triag_finger_tip_pub.publish(triangulated_point)

        if self.current_mode == OperationMode.WAYPOINT_REGISTRATION:
            self.process_waypoint_registration_gestures(dominant_left_gesture, dominant_right_gesture, triangulated_pose)
        elif self.current_mode == OperationMode.TRAJECTORY_TYPE_SELECTION:
            self.process_trajectory_type_selection_gestures(dominant_left_gesture, dominant_right_gesture, triangulated_pose)

    def process_waypoint_registration_gestures(self, dominant_left_gesture, dominant_right_gesture, triangulated_pose):
        """Process gestures in waypoint registration mode"""

        # Left hand Peace for clearing waypoints
        if dominant_left_gesture == 'Peace':
            if self.clear_peace_start_time is None:
                self.clear_peace_start_time = self.get_clock().now()

            peace_duration = (self.get_clock().now() - self.clear_peace_start_time).nanoseconds / 1e9
            if not self.clear_peace_triggered and peace_duration > 2.0:
                self.get_logger().info("Left hand 'Peace' detected, clearing waypoints and trajectory types.")
                self.publish_status_text("Clearing waypoints")
                self.waypoints = []
                self.trajectory_types = []
                self.interim_points = []
                self.interim_point_indices = []
                self.publish_waypoints()
                self.clear_peace_triggered = True
        else:
            self.clear_peace_start_time = None
            self.clear_peace_triggered = False

        # Right hand Hold for adding waypoints
        if dominant_right_gesture == 'Hold':
            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            hold_duration = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if not self.hold_triggered and hold_duration > 2.0:
                self.add_triangulated_waypoint(triangulated_pose)
                self.hold_triggered = True
        else:
            self.hold_start_time = None
            self.hold_triggered = False

        # Right hand Peace for executing waypoints (only if we have waypoints)
        if dominant_right_gesture == 'Peace':
            if self.execute_peace_start_time is None:
                self.execute_peace_start_time = self.get_clock().now()

            execute_peace_duration = (self.get_clock().now() - self.execute_peace_start_time).nanoseconds / 1e9
            if not self.execute_triggered and execute_peace_duration > 2.0 and self.waypoints:
                self.get_logger().info("Right hand 'Peace' detected in waypoint mode, executing trajectory.")
                self.publish_status_text("Executing trajectory")
                self.execute_waypoints()
                self.execute_triggered = True
        else:
            self.execute_peace_start_time = None
            self.execute_triggered = False

    def process_trajectory_type_selection_gestures(self, dominant_left_gesture, dominant_right_gesture, triangulated_pose):
        """Process gestures in trajectory type selection mode"""

        # If waiting for interim point, only accept Hold gesture
        if self.waiting_for_interim_point:
            if dominant_right_gesture == 'Hold':
                if self.hold_start_time is None:
                    self.hold_start_time = self.get_clock().now()

                hold_duration = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
                if not self.hold_triggered and hold_duration > 1.0:
                    self.add_interim_point(triangulated_pose)
                    self.hold_triggered = True
            else:
                self.hold_start_time = None
                self.hold_triggered = False
            return  # Don't process other gestures while waiting for interim point

        # Right hand Peace for PTP
        if dominant_right_gesture == 'Hold':
            if self.right_peace_start_time is None:
                self.right_peace_start_time = self.get_clock().now()

            duration = (self.get_clock().now() - self.right_peace_start_time).nanoseconds / 1e9
            if not self.right_peace_triggered and duration > 3.0:
                self.select_trajectory_type("PTP")
                self.right_peace_triggered = True
        else:
            self.right_peace_start_time = None
            self.right_peace_triggered = False

        # Left hand Peace for LIN
        if dominant_left_gesture == 'Peace':
            if self.left_peace_start_time is None:
                self.left_peace_start_time = self.get_clock().now()

            duration = (self.get_clock().now() - self.left_peace_start_time).nanoseconds / 1e9
            if not self.left_peace_triggered and duration > 3.0:
                self.select_trajectory_type("LIN")
                self.left_peace_triggered = True
        else:
            self.left_peace_start_time = None
            self.left_peace_triggered = False

        # Right hand Hold for CIRC
        if dominant_right_gesture == 'Pointer':
            if self.right_hold_start_time is None:
                self.right_hold_start_time = self.get_clock().now()

            duration = (self.get_clock().now() - self.right_hold_start_time).nanoseconds / 1e9
            if not self.right_hold_triggered and duration > 3.0:
                self.select_trajectory_type("CIRC")
                self.right_hold_triggered = True
        else:
            self.right_hold_start_time = None
            self.right_hold_triggered = False

    def _trinangulate_fingertip_pose(self):
        rays = []
        for name in self.camera_names:
            state = self.camera_states[name]
            if state['finger_tip_pose'] is not None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.world_frame,
                        state['finger_tip_pose'].header.frame_id,
                        rclpy.time.Time())

                    # TF transform utilities operate on PointStamped for point transforms.
                    camera_origin = PointStamped()
                    camera_origin.header.frame_id = state['finger_tip_pose'].header.frame_id
                    camera_origin.point.x = 0.0
                    camera_origin.point.y = 0.0
                    camera_origin.point.z = 0.0

                    # Convert the Vector3Stamped finger tip into a PointStamped so we can transform it
                    fingertip_point = state['finger_tip_pose']

                    camera_origin_world = tf2_geometry_msgs.do_transform_point(camera_origin, transform)
                    point_transformed = tf2_geometry_msgs.do_transform_point(fingertip_point, transform)

                    direction = np.array([
                        point_transformed.point.x - camera_origin_world.point.x,
                        point_transformed.point.y - camera_origin_world.point.y,
                        point_transformed.point.z - camera_origin_world.point.z
                    ])
                    direction = direction / np.linalg.norm(direction)

                    origin = np.array([camera_origin_world.point.x, camera_origin_world.point.y, camera_origin_world.point.z])
                    rays.append((origin, direction))

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(f"Could not transform point from {state['finger_tip_pose'].header.frame_id} to {self.world_frame}: {e}")

        if len(rays) < 2:
            self.get_logger().warn(f"Need at least 2 cameras to triangulate, but only have {len(rays)}. Cannot add waypoint.")
            return

        # Find the point that is closest to all rays
        estimated_point = self.find_closest_point_to_rays(rays)

        triangulated_pose = Pose()
        triangulated_pose.position.x = estimated_point[0]
        triangulated_pose.position.y = estimated_point[1]
        triangulated_pose.position.z = estimated_point[2]

        return triangulated_pose

    def add_triangulated_waypoint(self, triangulated_pose):
        self.waypoints.append(triangulated_pose)
        self.publish_waypoints()
        self.get_logger().info(f"Added waypoint #{len(self.waypoints)} at [{triangulated_pose.position.x:.3f}, {triangulated_pose.position.y:.3f}, {triangulated_pose.position.z:.3f}]")

        # After adding a waypoint, switch to trajectory type selection mode
        # (unless it's the first waypoint, which doesn't need a trajectory type)
        if len(self.waypoints) > 1:
            self.switch_mode()
            self.get_logger().info("Please select trajectory type for the segment just created.")
        else:
            self.get_logger().info("First waypoint added. Add another waypoint to create a trajectory segment.")

    def add_interim_point(self, triangulated_pose):
        """Add an interim point for a CIRC trajectory"""

        # Add the interim point and finalize the CIRC trajectory type
        segment_index = len(self.trajectory_types)
        self.interim_points.append(triangulated_pose)
        self.interim_point_indices.append(segment_index)
        self.trajectory_types.append(self.pending_trajectory_type)

        self.get_logger().info(f"Added interim point for CIRC trajectory at [{triangulated_pose.position.x:.3f}, {triangulated_pose.position.y:.3f}, {triangulated_pose.position.z:.3f}]")
        self.get_logger().info(f"Trajectory type '{self.pending_trajectory_type}' set for segment {segment_index + 1}")

        # Reset interim point waiting state
        self.waiting_for_interim_point = False
        self.pending_trajectory_type = None

        # Switch back to waypoint registration mode
        self.switch_mode()

    def select_trajectory_type(self, traj_type):
        """Select trajectory type for the last segment"""
        if len(self.waypoints) < 2:
            self.get_logger().warn("Need at least 2 waypoints to set a trajectory type.")
            return

        if traj_type == "CIRC":
            # For CIRC, we need an interim point
            self.get_logger().info(f"Selected trajectory type: {traj_type}. Please register an interim point using Hold gesture.")
            self.waiting_for_interim_point = True
            self.pending_trajectory_type = traj_type
        else:
            # For PTP and LIN, just add the type and switch back
            self.trajectory_types.append(traj_type)
            segment_index = len(self.trajectory_types)
            self.get_logger().info(f"Trajectory type '{traj_type}' set for segment {segment_index}")

            # Switch back to waypoint registration mode
            self.switch_mode()
        self.publish_status_text("Waypoint added")

    def find_closest_point_to_rays(self, rays):
        """
        Finds the point closest to a set of 3D rays.
        This is a least-squares problem to find the point that minimizes the sum
        of squared distances to each ray.
        """
        I = np.identity(3)
        A = np.zeros((3, 3))
        b = np.zeros(3)

        for p, n in rays:
            nnT = np.outer(n, n)
            A += I - nnT
            b += np.dot(I - nnT, p)

        try:
            # Solve the linear system A*q = b for q
            q = np.linalg.solve(A, b)
            return q
        except np.linalg.LinAlgError:
            self.get_logger().error("Could not find intersection point, matrix is singular.")
            # Fallback to averaging ray origins if the solver fails
            origins = [ray[0] for ray in rays]
            return np.mean(origins, axis=0)

    def publish_waypoints(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.world_frame
        pose_array_msg.poses = self.waypoints
        self.waypoints_publisher.publish(pose_array_msg)
        self.get_logger().debug(f"Published {len(self.waypoints)} waypoints.")

    def publish_status_text(self, text):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "status_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5  # Position it somewhere visible
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = text
        self.status_marker_publisher.publish(marker)

    def execute_waypoints(self):
        """Execute the waypoint trajectory using the action server"""
        if len(self.waypoints) < 2:
            self.get_logger().warn("Need at least 2 waypoints to execute a trajectory.")
            return

        if self.executing:
            self.get_logger().warn("Already executing a trajectory.")
            return

        # Fill in missing trajectory types with default "PTP"
        expected_types = len(self.waypoints) - 1
        while len(self.trajectory_types) < expected_types:
            self.trajectory_types.append("PTP")
            self.get_logger().info(f"Using default trajectory type 'PTP' for segment {len(self.trajectory_types)}")

        # Log the trajectory plan
        self.get_logger().info("=" * 50)
        self.get_logger().info("Executing trajectory with the following plan:")
        for i in range(len(self.trajectory_types)):
            traj_type = self.trajectory_types[i]
            info_str = f"  Segment {i+1}: Waypoint {i+1} -> Waypoint {i+2} using {traj_type}"
            if traj_type == "CIRC" and i in self.interim_point_indices:
                idx = self.interim_point_indices.index(i)
                interim = self.interim_points[idx]
                info_str += f" (interim: [{interim.position.x:.3f}, {interim.position.y:.3f}, {interim.position.z:.3f}])"
            self.get_logger().info(info_str)
        self.get_logger().info("=" * 50)

        # Create goal message
        goal_msg = ExecuteWayPointsWithTypes.Goal()

        # Create PoseArray for waypoints
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.world_frame
        pose_array.poses = self.waypoints

        goal_msg.waypoints = pose_array
        goal_msg.trajectory_types = self.trajectory_types
        goal_msg.interim_points = self.interim_points
        goal_msg.interim_point_indices = self.interim_point_indices

        # Send goal to action server
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal to action server...")
        self.executing = True

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.execute_feedback_callback
        )
        send_goal_future.add_done_callback(self.execute_goal_response_callback)

    def execute_goal_response_callback(self, future):
        """Callback when action server accepts/rejects the goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            self.executing = False
            return

        self.get_logger().info('Goal accepted by action server, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execute_result_callback)

    def execute_feedback_callback(self, feedback_msg):
        """Callback for action feedback"""
        feedback = feedback_msg.feedback
        phase_str = f"[{feedback.current_phase.upper()}]" if feedback.current_phase else ""
        segment_str = f"Segment {feedback.current_segment + 1}" if feedback.current_segment >= 0 else ""

        log_msg = f"{phase_str} {segment_str} {feedback.status} | Progress: {feedback.progress * 100:.1f}%"
        self.get_logger().info(log_msg.strip())

    def execute_result_callback(self, future):
        """Callback for action result"""
        result = future.result().result

        if result.success:
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"Trajectory execution succeeded! {result.message}")

            # Log segment results
            if len(result.segment_success) > 0:
                self.get_logger().info("Segment results:")
                for i, (success, msg) in enumerate(zip(result.segment_success, result.segment_messages)):
                    status = "✓" if success else "✗"
                    self.get_logger().info(f"  {status} Segment {i+1}: {msg}")
            self.get_logger().info("=" * 50)
        else:
            self.get_logger().error("=" * 50)
            self.get_logger().error(f"Trajectory execution failed! {result.message}")

            # Log which segments failed
            if len(result.segment_success) > 0:
                self.get_logger().error("Segment results:")
                for i, (success, msg) in enumerate(zip(result.segment_success, result.segment_messages)):
                    status = "✓" if success else "✗"
                    log_func = self.get_logger().info if success else self.get_logger().error
                    log_func(f"  {status} Segment {i+1}: {msg}")
            self.get_logger().error("=" * 50)

        self.executing = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
