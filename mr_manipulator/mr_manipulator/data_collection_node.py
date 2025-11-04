import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector_node')

        self.declare_parameter('number_of_classes', 2)
        self.number_of_classes = self.get_parameter('number_of_classes').get_parameter_value().integer_value
        self.get_logger().info(f'Number of classes to collect: {self.number_of_classes}')

        self.subscription = self.create_subscription(
            Image,
            '/image_rect',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.3)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.data = []
        self.labels = []
        self.collecting = False
        self.counter = 0
        self.frame_counter = 0
        self.current_class = 0

    def image_callback(self, msg):
        if self.current_class >= self.number_of_classes:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        frame = cv2.flip(frame, 1)
        self.frame_counter += 1

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for i in range(len(results.multi_hand_landmarks)):
                hand_landmarks = results.multi_hand_landmarks[i]

                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

                if self.collecting:
                    if self.frame_counter % 2 == 0:
                        data_aux = []
                        hand_world_landmarks = results.multi_hand_world_landmarks[i]
                        for landmark in hand_world_landmarks.landmark:
                            data_aux.append(landmark.x)
                            data_aux.append(landmark.y)

                        self.data.append(data_aux)
                        self.labels.append(self.current_class)
                        self.counter += 1

        if self.collecting:
            cv2.putText(frame, f"Class: {self.current_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, f"Size: {self.counter}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            cv2.putText(frame, "Press Q to start collecting data", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if self.collecting:
                self.get_logger().info(f'Stopped collecting data for class {self.current_class}')
                self.collecting = False
                self.current_class += 1
                self.counter = 0
                if self.current_class >= self.number_of_classes:
                    self.get_logger().info('All classes collected. Shutting down.')
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            else:
                self.get_logger().info(f'Started collecting data for class {self.current_class}')
                self.collecting = True
        elif key == ord('c') and cv2.getWindowProperty("Frame", cv2.WND_PROP_VISIBLE) < 1:
             rclpy.shutdown()


    def save_data(self):
        if not self.data:
            self.get_logger().info("No data collected to save.")
            return

        # Convert to numpy arrays
        data_np = np.array(self.data)
        labels_np = np.array(self.labels).reshape(-1, 1)

        # Combine labels and data
        dataset = np.hstack((labels_np, data_np))

        # Create header for the CSV file
        header = ['class']
        for i in range(21):
            header += [f'x{i}', f'y{i}']
        header_str = ','.join(header)

        # Save to CSV
        np.savetxt('data.csv', dataset, delimiter=',', header=header_str, fmt='%f', comments='')
        self.get_logger().info("Dataset created and saved as data.csv!")

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()