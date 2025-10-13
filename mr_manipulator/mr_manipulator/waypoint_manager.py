import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped, PoseArray, Pose, PointStamped
from std_srvs.srv import Trigger
from functools import partial
import tf2_ros
import tf2_geometry_msgs
from collections import Counter
import numpy as np

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

        # Publishers
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', 10)

        # Service Client
        self.execute_client = self.create_client(Trigger, '/execute_waypoints')

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
                Vector3Stamped, f'/{name}/finger_tip_pose', partial(self.finger_tip_callback, camera_name=name), 10)

            self.camera_states[name]['subs'].extend([left_gesture_sub, right_gesture_sub, finger_tip_sub])
            self.get_logger().info(f"Subscribed to topics for camera: {name}")

        self.waypoints = []

        # Global gesture tracking states
        self.hold_start_time = None
        self.hold_triggered = False
        self.peace_start_time = None
        self.peace_triggered = False
        self.execute_peace_start_time = None
        self.execute_triggered = False

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

    def process_gestures(self):
        dominant_left_gesture = self.get_dominant_gesture('left')
        dominant_right_gesture = self.get_dominant_gesture('right')

        # Left hand for clearing
        if dominant_left_gesture == 'Peace':
            if self.peace_start_time is None:
                self.peace_start_time = self.get_clock().now()

            peace_duration = (self.get_clock().now() - self.peace_start_time).nanoseconds / 1e9
            if not self.peace_triggered and peace_duration > 2.0:
                self.get_logger().info("Dominant 'Peace' from left hand detected, clearing waypoints.")
                self.waypoints = []
                self.publish_waypoints()
                self.peace_triggered = True
        else:
            self.peace_start_time = None
            self.peace_triggered = False

        # Right hand for adding waypoints
        if dominant_right_gesture == 'Hold':
            if self.hold_start_time is None:
                self.hold_start_time = self.get_clock().now()

            hold_duration = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9
            if not self.hold_triggered and hold_duration > 1.0:
                self.add_triangulated_waypoint()
                self.hold_triggered = True
        else:
            self.hold_start_time = None
            self.hold_triggered = False

        # Right hand for executing
        if dominant_right_gesture == 'Peace':
            if self.execute_peace_start_time is None:
                self.execute_peace_start_time = self.get_clock().now()

            execute_peace_duration = (self.get_clock().now() - self.execute_peace_start_time).nanoseconds / 1e9
            if not self.execute_triggered and execute_peace_duration > 2.0 and self.waypoints:
                self.get_logger().info("Dominant 'Peace' from right hand detected, executing waypoints.")
                request = Trigger.Request()
                future = self.execute_client.call_async(request)
                future.add_done_callback(self.execute_callback)
                self.execute_triggered = True
        else:
            self.execute_peace_start_time = None
            self.execute_triggered = False

    def add_triangulated_waypoint(self):
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
                    # Convert incoming Vector3Stamped to PointStamped for transforming.
                    camera_origin = PointStamped()
                    camera_origin.header.frame_id = state['finger_tip_pose'].header.frame_id
                    camera_origin.point.x = 0.0
                    camera_origin.point.y = 0.0
                    camera_origin.point.z = 0.0

                    # Convert the Vector3Stamped finger tip into a PointStamped so we can transform it
                    fingertip_point = PointStamped()
                    fingertip_point.header.frame_id = state['finger_tip_pose'].header.frame_id
                    fingertip_point.point.x = state['finger_tip_pose'].vector.x
                    fingertip_point.point.y = state['finger_tip_pose'].vector.y
                    fingertip_point.point.z = state['finger_tip_pose'].vector.z

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

        pose = Pose()
        pose.position.x = estimated_point[0]
        pose.position.y = estimated_point[1]
        pose.position.z = estimated_point[2]

        self.waypoints.append(pose)
        self.publish_waypoints()
        self.get_logger().info(f"Added triangulated waypoint at {estimated_point}")

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

    def execute_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

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
