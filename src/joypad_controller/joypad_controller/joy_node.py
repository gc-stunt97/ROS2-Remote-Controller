#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import json

class JoystickNode(Node):
    def __init__(self):
        super().__init__("joystick_node")
        self.serial_port = serial.Serial("/dev/ttyACM0", 57600)  # Configura la porta seriale
        self.publisher_left = self.create_publisher(Point, "left_joystick_data", 10)
        self.publisher_right = self.create_publisher(Point, "right_joystick_data", 10)
        self.timer = self.create_timer(0.01, self.publish_data)
        self.get_logger().info("Joystick node has been started")

    def parse_serial_data(self, line):
        try:
            data = json.loads(line)
          
            point_msg_left = Point()
            point_msg_left.x = float(data["LX"])
            point_msg_left.y = float(data["LY"])
            point_msg_left.z = float(data["LZ"])

            point_msg_right = Point()
            point_msg_right.x = float(data["RX"])
            point_msg_right.y = float(data["RY"])
            point_msg_right.z = float(data["RZ"])
            self.get_logger().info(line)
            return point_msg_left, point_msg_right
        except json.JSONDecodeError:
            return None, None

    def publish_data(self):
        line = self.serial_port.readline().decode("utf-8").strip()

        # Parsing dei dati seriali JSON e creazione dei messaggi Point
        point_msg_left, point_msg_right = self.parse_serial_data(line)

        if point_msg_left is not None:
            self.publisher_left.publish(point_msg_left)
        
        if point_msg_right is not None:
            self.publisher_right.publish(point_msg_right)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()