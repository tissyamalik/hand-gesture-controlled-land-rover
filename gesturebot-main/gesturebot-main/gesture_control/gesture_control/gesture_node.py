import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')

        # Publishers
        self.pub_bot1 = self.create_publisher(Twist, '/bot1/cmd_vel', 10)
        self.pub_bot2 = self.create_publisher(Twist, '/bot2/cmd_vel', 10)

        # Init MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.drawing = mp.solutions.drawing_utils

        # Start webcam (IP cam)
        self.cap = cap = cv2.VideoCapture("http://10.174.14.44:8080/video")

        # self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1, self.process_frame)

    def count_fingers(self, hand_landmarks):
        tips_ids = [4, 8, 12, 16, 20]
        fingers = []

        if hand_landmarks.landmark[tips_ids[0]].x < hand_landmarks.landmark[tips_ids[0] - 1].x:
            fingers.append(1)
        else:
            fingers.append(0)

        for id in range(1, 5):
            if hand_landmarks.landmark[tips_ids[id]].y < hand_landmarks.landmark[tips_ids[id] - 2].y:
                fingers.append(1)
            else:
                fingers.append(0)

        return fingers.count(1)

    def get_command(self, finger_count):
        cmd = Twist()
        if finger_count == 0:  # stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif finger_count == 1:  # forward
            cmd.linear.x = 0.5
        elif finger_count == 2:  # left
            cmd.angular.z = 0.5
        elif finger_count == 3:  # right
            cmd.angular.z = -0.5
        elif finger_count >= 4:  # reverse
            cmd.linear.x = -0.5
        return cmd

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (320, 240))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        cmd1 = Twist()
        cmd2 = Twist()

        if result.multi_hand_landmarks and result.multi_handedness:
            for hand_landmarks, handedness in zip(result.multi_hand_landmarks, result.multi_handedness):
                label = handedness.classification[0].label
                finger_count = self.count_fingers(hand_landmarks)
                cmd = self.get_command(finger_count)

                if label == "Left":
                    cmd1 = cmd
                elif label == "Right":
                    cmd2 = cmd

                self.drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        self.pub_bot1.publish(cmd1)
        self.pub_bot2.publish(cmd2)

        # Debug info
        cv2.imshow("Gesture Control", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
