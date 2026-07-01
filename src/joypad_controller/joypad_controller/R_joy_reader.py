#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class R_JoystickSubscriber(Node):
    def __init__(self):
        super().__init__("R_joystick_subscriber")
        self.subscription = self.create_subscription(Point, "right_joystick_data", self.callback, 10)
        self.get_logger().info("Joystick subscriber has been started")

    def callback(self, msg):
        self.get_logger().info(f"Received joystick data - X: {msg.x}, Y: {msg.y}, Z: {msg.z}")

def main(args=None):
    rclpy.init(args=args)
    node = R_JoystickSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()