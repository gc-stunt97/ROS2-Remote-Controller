#!/usr/bin/env python3
"""Avvia il controller per la base mobile AIRA: nodo joystick + dashboard AIRA.

Riusa il publisher joystick dello STM32 (pacchetto joypad_controller, condiviso con
RobotHex) e ci mette sopra la dashboard dedicata alla base mobile.

    ros2 launch aira_dashboard aira.launch.py

⚠️ La GUI vuole uno schermo: lanciarla dal display 7" del controller, oppure da SSH con
`export DISPLAY=:0`. Chiudendo la dashboard si spegne anche il nodo joystick.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0",
                              description="seriale dello STM32 (joystick)"),
        Node(
            package="joypad_controller",
            executable="joypad_node",
            name="joystick_node",
            output="screen",
            parameters=[{"serial_port": serial_port}],
        ),
        Node(
            package="aira_dashboard",
            executable="aira_dashboard",
            name="aira_dashboard",
            output="screen",
            on_exit=Shutdown(),
        ),
    ])
