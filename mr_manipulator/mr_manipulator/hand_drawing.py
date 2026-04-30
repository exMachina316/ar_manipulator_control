import pickle
import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

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
        self.declare_parameter('model_path', "/root/ur_ws/src/mr_manipulator/models/xgboost_model.2.0.0.p")
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Load the hand gesture model
        with open(model_path, 'rb') as f:
            self.model = pickle.load(f)

        # Create ROS 2 publishers
        self.left_gesture_publisher = self.create_publisher(String, 'left_hand_gesture', 10)
        self.right_gesture_publisher = self.create_publisher(String, 'right_hand_gesture', 10)
        self.finger_tip_publisher = self.create_publisher(PointStamped, 'finger_tip_pose', 10)

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

        self.labels_dict = {0: "Pointer", 1: "Peace", 2: "Hold"}
        # self.labels_dict = {0: "Pointer", 1: "Peace", 2: "Thumbs Up", 3: "Thumbs Down", 4: "Hold"}
        self.status_text = ""

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.get_logger().info("Camera info received.")
        # We can unsubscribe after receiving it once if it's static
        self.destroy_subscription(self.camera_info_subscription)
        self.camera_info_subscription = None

    def image_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        H, W, _ = frame.shape
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

                # take only the x and y coordinates of the world landmarks (relative to camera center)
                data_aux = []
                hand_world_landmarks = results.multi_hand_world_landmarks[i]
                for landmark in hand_world_landmarks.landmark:
                    data_aux.append(landmark.x)
                    data_aux.append(landmark.y)

                # make gesture prediction using pretrained model
                data_aux = np.array(data_aux).reshape(1, -1)
                prediction = self.model.predict(data_aux)[0]
                predicted_label = self.labels_dict.get(prediction, "Unknown")

                gesture_msg = String()
                gesture_msg.data = predicted_label

                if handedness == 'Left':
                    self.left_gesture_publisher.publish(gesture_msg)
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
                        index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                        
                        self.get_logger().info(f"Gotten to the camera info callback, camera info: {self.camera_info}")
                        if self.camera_info:
                            fx = self.camera_info.p[0]  # focal length x
                            fy = self.camera_info.p[5]  # focal length y
                            cx = self.camera_info.p[2]  # camera center x
                            cy = self.camera_info.p[6]  # camera center y

                            # Normalized camera frame ray pointing to fingertip
                            u = index_finger_tip.x * W
                            v = index_finger_tip.y * H
                            self.get_logger().info(f"Index finger tip pixel coordinates: (u={index_finger_tip.x * W}, v={index_finger_tip.y * H})")

                            x_coord = (u - cx) / fx
                            y_coord = (v - cy) / fy

                            # Publish finger tip pose
                            point_stamped_msg = PointStamped()
                            point_stamped_msg.header.stamp = self.get_clock().now().to_msg()
                            point_stamped_msg.header.frame_id = msg.header.frame_id

                            # Unit vector in camera frame pointing to fingertip
                            point_stamped_msg.point.x = x_coord
                            point_stamped_msg.point.y = y_coord
                            point_stamped_msg.point.z = 1.0

                            self.finger_tip_publisher.publish(point_stamped_msg)

                if handedness == 'Right':
                    self.right_gesture_publisher.publish(gesture_msg)
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

        cv2.imshow('Hand Drawing', frame)

        cv_key = cv2.waitKey(1)
        if cv_key & 0xFF == ord('q'):
            self.cleanup()

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
