#!/bin/bash
# Avvia il controller RobotHex (nodo joystick + GUI) con un solo comando.
# Autocontenuto: imposta DISPLAY e fa il source dell'ambiente ROS da solo,
# cosi' funziona sia lanciato dall'icona del desktop sia da SSH.
export DISPLAY="${DISPLAY:-:0}"          # da SSH non c'e' DISPLAY -> usa lo schermo :0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
ros2 launch joypad_controller joypad.launch.py
