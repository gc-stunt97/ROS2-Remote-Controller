"""Launch del "modo mouse": lo stick destro pilota il cursore di Ubuntu.

Uso:
    ros2 launch joypad_controller mouse.launch.py
    ros2 launch joypad_controller mouse.launch.py stick:=left speed:=1200.0

Avvia il publisher joystick (`joypad_node`, l'unico proprietario della seriale) e
il `cursor_node`, che traduce lo stick in un mouse virtuale uinput.

⚠️ NON si lancia insieme a una plancia: si contenderebbero /dev/aira_controller,
esattamente come le due plance fra loro. Gli script in `desktop/` si passano il
testimone da soli (il modo mouse si fa da parte quando apri una plancia e torna
quando la chiudi), quindi in uso normale non ci pensi.

Prerequisiti una tantum: vedi udev/99-aira-uinput.rules (python3-evdev + regola
udev + gruppo 'input').
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    stick = LaunchConfiguration("stick")
    speed = LaunchConfiguration("speed")

    args = [
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/aira_controller",
            description="Porta seriale della Blue Pill (alias udev o /dev/ttyACM*)"),
        DeclareLaunchArgument(
            "stick", default_value="right",
            description="Quale stick muove il cursore: 'right' o 'left'"),
        DeclareLaunchArgument(
            "speed", default_value="900.0",
            description="Velocita' del cursore a fondo corsa, in px/s"),
    ]

    joystick_node = Node(
        package="joypad_controller",
        executable="joypad_node",
        name="joystick_node",
        output="screen",
        parameters=[{"serial_port": serial_port}],
    )

    cursor_node = Node(
        package="joypad_controller",
        executable="cursor_node",
        name="cursor_node",
        output="screen",
        parameters=[{"stick": stick, "speed": speed}],
    )

    return LaunchDescription(args + [joystick_node, cursor_node])
