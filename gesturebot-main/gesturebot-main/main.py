import cv2
import mediapipe as mp
import socket

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

HOST = '127.0.0.1'  
PORT = 5005              

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Start IP Webcam
cap = cv2.VideoCapture("http://10.174.14.44:8080/video")
# cap = cv2.VideoCapture(0)

def send_command(command, bot='bot1'):
    message = f"{bot}:{command}"
    print(f"Sending command: {message}")
    sock.sendto(message.encode(), (HOST, PORT))

def count_fingers(hand_landmarks):
    tips_ids = [4, 8, 12, 16, 20]
    fingers = []

    # Thumb (sideways check)
    if hand_landmarks.landmark[tips_ids[0]].x < hand_landmarks.landmark[tips_ids[0] - 1].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Other fingers
    for id in range(1, 5):
        if hand_landmarks.landmark[tips_ids[id]].y < hand_landmarks.landmark[tips_ids[id] - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    return fingers.count(1)

def get_command(finger_count):
    if finger_count == 0:
        return "stop"
    elif finger_count == 1:
        return "forward"
    elif finger_count == 2:
        return "left"
    elif finger_count == 3:
        return "right"
    elif finger_count >= 4:
        return "reverse"
    else:
        return "none"

with mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.5,model_complexity=0) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (320, 240))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        command_left = "none"
        command_right = "none"

        if result.multi_hand_landmarks and result.multi_handedness:
            for hand_landmarks, handedness in zip(result.multi_hand_landmarks, result.multi_handedness):
                label = handedness.classification[0].label  # 'Left' or 'Right'
                finger_count = count_fingers(hand_landmarks)
                command = get_command(finger_count)

                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                if label == "Left":
                    command_left = command
                elif label == "Right":
                    command_right = command

        # Print both commands
        print(f"Left Hand: {command_left} | Right Hand: {command_right}")

        send_command(command_left, bot='bot1')
        send_command(command_right, bot='bot2')

        # Show on frame
        cv2.putText(frame, f"Left: {command_left}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.putText(frame, f"Right: {command_right}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Gesture control", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

cap.release()
cv2.destroyAllWindows()
