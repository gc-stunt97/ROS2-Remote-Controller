#!/usr/bin/env python3
"""Banco dei micro AIRA: nodo joystick + programma di banco (CONTROLLER_HANDBOOK sez. 14).

    ros2 launch aira_bench bench.launch.py
    ros2 launch aira_bench bench.launch.py repo:=/home/giulio/AIRA_Robot

Il profilo del micro si legge dal clone di AIRA_Robot (default ~/AIRA_Robot, in sola lettura:
`git pull` per aggiornarlo). Chiudendo la finestra si spegne anche il nodo joystick.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    repo = LaunchConfiguration("repo")

    joystick_node = Node(
        package="joypad_controller", executable="joypad_node", name="joystick_node",
        output="screen", parameters=[{"serial_port": serial_port}],
    )
    bench_node = Node(
        package="aira_bench", executable="aira_bench", name="aira_bench",
        output="screen", arguments=["--repo", repo],
    )
    stop_on_close = RegisterEventHandler(
        OnProcessExit(target_action=bench_node, on_exit=[EmitEvent(event=Shutdown())]))

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/aira_controller"),
        DeclareLaunchArgument("repo", default_value=os.path.expanduser("~/AIRA_Robot")),
        joystick_node,
        bench_node,
        stop_on_close,
    ])
