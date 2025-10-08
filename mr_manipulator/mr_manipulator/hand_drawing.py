import pickle
import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
class HandDrawingNode(Node):
    def __init__(self):
        super().__init__('hand_drawing_node')

        cv2.namedWindow('Hand Drawing', cv2.WINDOW_NORMAL)

        # Declare and get the ROS 2 parameter for the model path
        self.declare_parameter('model_path', "/root/ur_ws/src/mr_manipulator/models/xgboost_model.p")
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Load the hand gesture model
        with open(model_path, 'rb') as f:
            self.model = pickle.load(f)

        sensor_qos = rclpy.qos.qos_profile_sensor_data

        # Create ROS 2 publisher for waypoints as PoseArray
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', sensor_qos)
        self.marker_publisher = self.create_publisher(Marker, 'hand_marker', sensor_qos)

        # Create ROS 2 client for executing waypoints
        self.execute_client = self.create_client(Trigger, 'execute_waypoints')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a subscriber to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)

        self.camera_info = None
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Initialize Mediapipe Hands
        self.hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.9)

        # Drawing and erasing configurations
        self.drawing_color = (0, 0, 255)
        self.waypoint_color = (0, 255, 0)
        self.canvas = None

        self.waypoints = []
        self.labels_dict = {0: 'Hold', 1: 'Pointer', 2: 'Peace'}

        # Hold gesture tracking
        self.hold_start_time = None
        self.hold_triggered = False
        self.index_finger_tip = None
        self.execute_triggered = False

        # Peace gesture tracking
        self.peace_start_time = None
        self.peace_triggered = False
        self.execute_peace_start_time = None

        self.status_text = ""

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.get_logger().info("Camera info received.")
        # We can unsubscribe after receiving it once if it's static
        self.destroy_subscription(self.camera_info_subscription)
        self.camera_info_subscription = None

    def execute_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def image_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # frame = cv2.flip(frame, 1)
        H, W, _ = frame.shape

        if self.canvas is None:
            self.canvas = np.zeros_like(frame)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if not results.multi_hand_landmarks:
            self.status_text = ""

        if results.multi_hand_landmarks:
            for i in range(len(results.multi_hand_landmarks)):
                hand_landmarks = results.multi_hand_landmarks[i]
                handedness = results.multi_handedness[i].classification[0].label

                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                data_aux = []
                hand_world_landmarks = results.multi_hand_world_landmarks[i]
                for landmark in hand_world_landmarks.landmark:
                    data_aux.append(landmark.x)
                    data_aux.append(landmark.y)

                data_aux = np.array(data_aux).reshape(1, -1)
                prediction = self.model.predict(data_aux)[0]
                # Convert to NumPy array & predict
                predicted_label = self.labels_dict.get(prediction, "Unknown")

                if handedness == 'Left':
                    # Display Prediction on Frame
                    font = cv2.FONT_HERSHEY_DUPLEX
                    font_scale = 1.5
                    thickness = 3
                    color = (0, 255, 0)
                    outline_color = (0, 0, 0)
                    (text_width, text_height), baseline = cv2.getTextSize(predicted_label, font, font_scale, thickness)
                    x = W - text_width - 30
                    y = 70
                    # Draw background rectangle for contrast
                    cv2.rectangle(frame, (x - 10, y - text_height - 10), (x + text_width + 10, y + baseline + 10), (255, 255, 255), -1)
                    # Draw outline for better visibility
                    cv2.putText(frame, predicted_label, (x, y), font, font_scale, outline_color, thickness + 2, cv2.LINE_AA)
                    # Draw main text
                    cv2.putText(frame, predicted_label, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

                    if predicted_label == 'Pointer':
                        self.index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                        self.peace_start_time = None
                        self.peace_triggered = False

                        # Publish marker for index finger tip
                        if self.camera_info:
                            fx = self.camera_info.k[0]
                            fy = self.camera_info.k[4]
                            cx = self.camera_info.k[2]
                            cy = self.camera_info.k[5]

                            u = self.index_finger_tip.x * W
                            v = self.index_finger_tip.y * H

                            z = 0.860  # a fixed depth
                            x = (u - cx) * z / fx
                            y = (v - cy) * z / fy

                            marker = Marker()
                            marker.header.frame_id = msg.header.frame_id
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.ns = "hand"
                            marker.id = 0
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            marker.pose.position.x = x
                            marker.pose.position.y = y
                            marker.pose.position.z = z
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.05
                            marker.scale.y = 0.05
                            marker.scale.z = 0.05
                            marker.color.a = 1.0
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                            self.marker_publisher.publish(marker)

                    elif predicted_label == 'Peace':
                        if self.peace_start_time is None:
                            self.peace_start_time = self.get_clock().now()

                        peace_duration = (self.get_clock().now() - self.peace_start_time).nanoseconds / 1e9

                        if not self.peace_triggered and peace_duration > 2.0:
                            self.get_logger().info("Peace sign detected for 2 seconds, clearing waypoints and canvas.")
                            self.waypoints = []
                            self.canvas = np.zeros_like(frame)
                            self.peace_triggered = True
                            self.status_text = "Cleared"
                    else:
                        self.index_finger_tip = None
                        self.peace_start_time = None
                        self.peace_triggered = False

                if handedness == 'Right':
                    # Display Prediction on Frame
                    font = cv2.FONT_HERSHEY_DUPLEX
                    font_scale = 1.5
                    thickness = 3
                    color = (0, 255, 0)
                    outline_color = (0, 0, 0)
                    (text_width, text_height), baseline = cv2.getTextSize(predicted_label, font, font_scale, thickness)
                    x = 30
                    y = 70
                    # Draw background rectangle for contrast
                    cv2.rectangle(frame, (x - 10, y - text_height - 10), (x + text_width + 10, y + baseline + 10), (255, 255, 255), -1)
                    # Draw outline for better visibility
                    cv2.putText(frame, predicted_label, (x, y), font, font_scale, outline_color, thickness + 2, cv2.LINE_AA)
                    # Draw main text
                    cv2.putText(frame, predicted_label, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

                    if predicted_label == 'Hold':
                        self.execute_triggered = False
                        self.execute_peace_start_time = None

                        if self.hold_start_time is None:
                            self.hold_start_time = self.get_clock().now()

                        hold_duration = (self.get_clock().now() - self.hold_start_time).nanoseconds / 1e9

                        if not self.hold_triggered and hold_duration > 1.0:
                            self.get_logger().info("Hold detected for 1 second, registering waypoint.")
                            if self.index_finger_tip is not None:
                                self.waypoints.append((self.index_finger_tip.x * W, self.index_finger_tip.y * H))
                                cv2.circle(self.canvas, (int(self.index_finger_tip.x * W), int(self.index_finger_tip.y * H)), 10, self.waypoint_color, -1)
                                self.status_text = "Added"
                            else:
                                self.get_logger().warn("Index finger tip not detected, cannot register waypoint.")

                            self.hold_triggered = True

                    elif predicted_label == 'Peace':
                        self.hold_start_time = None
                        self.hold_triggered = False

                        if self.execute_peace_start_time is None:
                            self.execute_peace_start_time = self.get_clock().now()

                        execute_peace_duration = (self.get_clock().now() - self.execute_peace_start_time).nanoseconds / 1e9

                        if not self.execute_triggered and execute_peace_duration > 2.0 and self.waypoints:
                            self.status_text = "Executing"
                            self.get_logger().info("Executing waypoints.")
                            request = Trigger.Request()
                            future = self.execute_client.call_async(request)
                            future.add_done_callback(self.execute_callback)

                            self.execute_triggered = True

                    else:
                        self.hold_start_time = None
                        self.hold_triggered = False
                        self.execute_triggered = False
                        self.execute_peace_start_time = None
                        self.status_text = ""

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = msg.header.frame_id

        if self.waypoints:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            for waypoint in self.waypoints:
                u = waypoint[0]
                v = waypoint[1]

                pose = Pose()

                # Convert pixel coordinates to camera frame coordinates
                z = 0.860 # a fixed depth
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose_array_msg.poses.append(pose)

        self.get_logger().debug(f"Published {len(pose_array_msg.poses)} waypoints.")
        self.waypoints_publisher.publish(pose_array_msg)

        if self.status_text:
            text = self.status_text
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 1.5
            thickness = 3
            color = (255, 0, 255)
            outline_color = (0, 0, 0)
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
            x = W // 2 - text_width // 2
            y = H - 30

            # Draw background rectangle for contrast
            cv2.rectangle(frame, (x - 10, y - text_height - 10), (x + text_width + 10, y + baseline + 10), (255, 255, 255), -1)

            # Draw outline for better visibility
            cv2.putText(frame, text, (x, y), font, font_scale, outline_color, thickness + 2, cv2.LINE_AA)
            # Draw main text
            cv2.putText(frame, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)

        frame_with_drawing = cv2.addWeighted(frame, 0.5, self.canvas, 0.5, 0)
        cv2.imshow('Hand Drawing', frame_with_drawing)

        cv_key = cv2.waitKey(1)
        if cv_key & 0xFF == ord('q'):
            self.cleanup()
        elif cv_key & 0xFF == ord('c'):
            self.get_logger().info("Clearing waypoints and canvas.")
            self.waypoints = []
            self.canvas = np.zeros_like(frame)
            self.status_text = "Cleared"
        elif cv_key & 0xFF == ord('e'):
            if self.waypoints:
                self.get_logger().info("Executing waypoints.")
                request = Trigger.Request()
                future = self.execute_client.call_async(request)
                future.add_done_callback(self.execute_callback)

    def cleanup(self):
        cv2.destroyAllWindows()
        self.get_logger().info("Shutting down Hand Drawing Node.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandDrawingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
