import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import time

COMMAND_MAP = {
    'stop': (0.0, 0.0),
    'forward': (0.5, 0.0),
    'reverse': (-0.5, 0.0),
    'left': (0.0, 0.5),
    'right': (0.0, -0.5)
}

class GestureListener(Node):
    def __init__(self):
        super().__init__('gesture_listener')

        self.publisher_bot1 = self.create_publisher(Twist, '/bot1/cmd_vel', 10)
        self.publisher_bot2 = self.create_publisher(Twist, '/bot2/cmd_vel', 10)

        self.last_command_time = {
            'bot1': time.time(),
            'bot2': time.time()
        }

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))
        self.get_logger().info("Socket bound to 0.0.0.0:5005")

        threading.Thread(target=self.listen_loop, daemon=True).start()
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info("Gesture listener running with timeout watchdog.")

    def listen_loop(self):
        self.get_logger().info("UDP listener thread started.")
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode().strip()
                self.get_logger().info(f"Received raw message from {addr}: {message}")

                bot, command = message.split(":")
                bot = bot.strip()
                command = command.strip()

                if command not in COMMAND_MAP:
                    self.get_logger().warn(f"Unknown command: {command}")
                    continue
                if bot not in self.last_command_time:
                    self.get_logger().warn(f"Unknown bot: {bot}")
                    continue

                self.last_command_time[bot] = time.time()
                self.get_logger().info(f"Command for {bot}: {command}")
                self.publish_command(bot, command)

            except Exception as e:
                self.get_logger().error(f"Error processing message: {e}")

    def publish_command(self, bot, command):
        linear_x, angular_z = COMMAND_MAP[command]
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        if bot == 'bot1':
            self.publisher_bot1.publish(msg)
        elif bot == 'bot2':
            self.publisher_bot2.publish(msg)

    def check_timeout(self):
        now = time.time()
        for bot in ['bot1', 'bot2']:
            if now - self.last_command_time[bot] > 1.0:
                self.publish_command(bot, 'stop')

def main(args=None):
    rclpy.init(args=args)
    node = GestureListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
