import cv2
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import numpy as np

# MediaPipe initialization
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Class Labels
CLASS_NAMES = {1:"Fist", 2: "Pointer", 3: "Peace", 4: "Three-finger", 5: "Open Palm"}

hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.8)

def get_distance_2d(landmark1, landmark2):
    return ((landmark1.x - landmark2.x) ** 2 +
            (landmark1.y - landmark2.y) ** 2) ** 0.5

def get_angle_2d(landmark1, landmark2):
    return np.arctan2(landmark2.y - landmark1.y, landmark2.x - landmark1.x)

def is_finger_extended(tip, pip, mcp, threshold=0.8):
    tip_t = get_angle_2d(tip, pip)
    pip_t = get_angle_2d(pip, mcp)

    if abs(tip_t - pip_t) < threshold:
        return 1
    return 0


def get_gesture_prediction(hand_world_landmarks: landmark_pb2.LandmarkList):
    index_finger_tip = hand_world_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_pip = hand_world_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    index_finger_mcp = hand_world_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]

    middle_finger_tip = hand_world_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_pip = hand_world_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    middle_finger_mcp = hand_world_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

    ring_finger_tip = hand_world_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_finger_pip = hand_world_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
    ring_finger_mcp = hand_world_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]

    pinky_finger_tip = hand_world_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    pinky_finger_pip = hand_world_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]
    pinky_finger_mcp = hand_world_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]

    is_index_extended = is_finger_extended(index_finger_tip, index_finger_pip, index_finger_mcp)
    is_middle_extended = is_finger_extended(middle_finger_tip, middle_finger_pip, middle_finger_mcp)
    is_ring_extended = is_finger_extended(ring_finger_tip, ring_finger_pip, ring_finger_mcp)
    is_pinky_extended = is_finger_extended(pinky_finger_tip, pinky_finger_pip, pinky_finger_mcp)

    if is_index_extended and not is_middle_extended and not is_ring_extended and not is_pinky_extended:
        return 2  # Pointer gesture
    elif is_index_extended and is_middle_extended and not is_ring_extended and not is_pinky_extended:
        return 3  # Peace gesture
    elif is_index_extended and is_middle_extended and is_ring_extended and not is_pinky_extended:
        return 4  # Three finger gesture
    elif is_index_extended and is_middle_extended and is_ring_extended and is_pinky_extended:
        return 5  # Open Palm gesture
    elif not is_index_extended and not is_middle_extended and not is_ring_extended and not is_pinky_extended:
        return 1  # Fist gesture

    return 0  # No recognized gesture

if __name__ == "__main__":
    # Start capturing video
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            break

        # Convert frame to RGB
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        # Draw hand landmarks
        if results.multi_hand_landmarks:
            for i in range(len(results.multi_hand_landmarks)):
                hand_landmarks = results.multi_hand_landmarks[i]
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                hand_world_landmarks = results.multi_hand_world_landmarks[i]
                prediction = get_gesture_prediction(hand_world_landmarks)
                prediction_label = CLASS_NAMES.get(prediction, "Unknown")

                # Display Prediction on Frame
                cv2.putText(frame, prediction_label, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show frame
        cv2.imshow("Live Gesture Recognition", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
