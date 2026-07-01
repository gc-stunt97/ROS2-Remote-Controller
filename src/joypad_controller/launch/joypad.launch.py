"""Launch del controller RobotHex: avvia insieme il publisher joystick e la GUI.

Uso:
    ros2 launch joypad_controller joypad.launch.py

Chiudendo la finestra della GUI si ferma anche il nodo joystick (vedi l'event
handler in fondo): "chiudo la finestra = spengo tutto".
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    joystick_node = Node(
        package="joypad_controller",
        executable="joypad_node",
        name="joystick_node",
        output="screen",
    )

    gui_node = Node(
        package="joypad_controller",
        executable="joypad_gui_app",
        name="joystick_gui",
        output="screen",
    )

    # Quando la GUI viene chiusa, spegni tutto il launch (quindi anche joystick_node).
    stop_on_gui_close = RegisterEventHandler(
        OnProcessExit(target_action=gui_node, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([joystick_node, gui_node, stop_on_gui_close])
