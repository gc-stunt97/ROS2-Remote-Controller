#!/usr/bin/env python3

import tkinter as tk
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point
import rclpy

class JoystickSubscriberApp:
    def __init__(self):
        self.node = rclpy.create_node('joystick_gui_subscriber')

        self.right_values = Point()
        self.left_values = Point()

        self.right_subscriber = self.node.create_subscription(
            Point, '/right_joystick_data', self.right_callback, QoSProfile(depth=10))
        self.left_subscriber = self.node.create_subscription(
            Point, '/left_joystick_data', self.left_callback, QoSProfile(depth=10))

        self.root = tk.Tk()
        self.root.title('Joystick Values')
        self.root.geometry("800x550")

        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side='left', padx=100)

        self.left_label = tk.Label(self.left_frame, text='Left Joystick:')
        self.left_label.pack(pady=(100, 0))

        self.left_frame.columnconfigure(0, weight=1)

        left_label_x = tk.Label(self.left_frame, text='LX:')
        left_label_x.pack()

        self.left_cursor_x = tk.Scale(self.left_frame, from_=1.0, to=-1.0, resolution=0.01, orient='vertical', length=150)
        self.left_cursor_x.pack()

        left_label_x = tk.Label(self.left_frame, text='LY:')
        left_label_x.pack()

        self.left_cursor_y = tk.Scale(self.left_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', length=150)
        self.left_cursor_y.pack()

        left_label_x = tk.Label(self.left_frame, text='LZ:')
        left_label_x.pack()

        self.left_cursor_z = tk.Scale(self.left_frame, from_=-2.0, to=2.0, resolution=0.01, orient='horizontal', length=150)
        self.left_cursor_z.pack()

        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side='right', padx=100)

        self.right_label = tk.Label(self.right_frame, text='Right Joystick:')
        self.right_label.pack(pady=(100, 0))

        self.right_frame.columnconfigure(0, weight=1)

        left_label_x = tk.Label(self.right_frame, text='RX:')
        left_label_x.pack()

        self.right_cursor_x = tk.Scale(self.right_frame, from_=1.0, to=-1.0, resolution=0.01, orient='vertical', length=150)
        self.right_cursor_x.pack()

        left_label_x = tk.Label(self.right_frame, text='RY:')
        left_label_x.pack()

        self.right_cursor_y = tk.Scale(self.right_frame, from_=-1.0, to=1.0, resolution=0.01, orient='horizontal', length=150)
        self.right_cursor_y.pack()

        left_label_x = tk.Label(self.right_frame, text='RZ:')
        left_label_x.pack()

        self.right_cursor_z = tk.Scale(self.right_frame, from_=-2.0, to=2.0, resolution=0.01, orient='horizontal', length=150)
        self.right_cursor_z.pack()

        self.update_gui_values()
        self.node.create_timer(0.1, self.update_gui_values)

    def right_callback(self, msg):
        self.right_values = msg

    def left_callback(self, msg):
        self.left_values = msg

    def update_gui_values(self):
        self.left_cursor_x.set(self.left_values.x)
        self.left_cursor_y.set(self.left_values.y)
        self.left_cursor_z.set(self.left_values.z)

        self.right_cursor_x.set(self.right_values.x)
        self.right_cursor_y.set(self.right_values.y)
        self.right_cursor_z.set(self.right_values.z)

        self.root.update_idletasks()

    def run(self):
        rclpy.spin(self.node)


def main(args=None):
    rclpy.init(args=args)
    app = JoystickSubscriberApp()
    app.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()