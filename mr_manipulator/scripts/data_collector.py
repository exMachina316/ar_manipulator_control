import cv2
import mediapipe as mp
import numpy as np

NUMBER_OF_CLASSES = int(input("Enter number of classes to collect: "))

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.3)

data, labels = [], []
cap = cv2.VideoCapture(0)

for j in range(NUMBER_OF_CLASSES):

    collecting = False
    counter = 0
    frame_counter = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        frame_counter += 1

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for i in range(len(results.multi_hand_landmarks)):
                hand_landmarks = results.multi_hand_landmarks[i]

                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style(),
                )

                if collecting:
                    if frame_counter % 2 == 0:
                        data_aux = []
                        hand_world_landmarks = results.multi_hand_world_landmarks[i]
                        for landmark in hand_world_landmarks.landmark:
                            data_aux.append(landmark.x)
                            data_aux.append(landmark.y)

                        data.append(data_aux)
                        labels.append(j)
                        counter+=1

        if collecting:
            cv2.putText(frame, f"Class: {j}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, f"Size: {counter}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            cv2.putText(frame, "Press Q to start collecting data", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


        cv2.imshow("Frame", frame)
        # Display at 30 FPS
        if cv2.waitKey(1) & 0xFF == ord('q'):
            if collecting:
                print(f'Stopped collecting data for class {j}')
                break
            else:
                print(f'Started collecting data for class {j}')
                collecting = True

cap.release()
cv2.destroyAllWindows()

# Convert to numpy arrays
data = np.array(data)
labels = np.array(labels).reshape(-1, 1)

# Combine labels and data
dataset = np.hstack((labels, data))

# Create header for the CSV file
header = ['class']
for i in range(21):
    header += [f'x{i}', f'y{i}']
header_str = ','.join(header)

# Save to CSV
np.savetxt('data.csv', dataset, delimiter=',', header=header_str, fmt='%f', comments='')


print("Dataset created and saved as data.csv!")