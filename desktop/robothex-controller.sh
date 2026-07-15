#!/bin/bash
# Avvia il controller RobotHex (nodo joystick + GUI) con un solo comando.
# Autocontenuto: imposta DISPLAY e fa il source dell'ambiente ROS da solo,
# cosi' funziona sia lanciato dall'icona del desktop sia da SSH.
export DISPLAY="${DISPLAY:-:0}"          # da SSH non c'e' DISPLAY -> usa lo schermo :0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# Il modo mouse tiene la seriale: si fa da parte prima che parta la plancia, e
# torna da solo quando la chiudi. Se restassero accesi entrambi si ruberebbero i
# byte a vicenda e non funzionerebbe NESSUNO dei due, senza dare errori: sembra
# che gli stick siano morti. Vedi mouse-mode.sh.
[ -x "$HOME/mouse-mode.sh" ] && "$HOME/mouse-mode.sh" stop

ros2 launch joypad_controller joypad.launch.py

# Plancia chiusa: il telecomando torna a fare da mouse.
[ -x "$HOME/mouse-mode.sh" ] && "$HOME/mouse-mode.sh" start
