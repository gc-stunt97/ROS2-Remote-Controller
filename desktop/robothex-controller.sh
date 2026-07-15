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

# ⚠️ Pulizia dei residui, PRIMA di ridare la seriale al modo mouse.
# Se il launch muore male (crash, kill), i suoi nodi NON muoiono con lui: restano
# orfani e continuano a tenere /dev/aira_controller. Senza questa riga il modo
# mouse non ripartirebbe piu' (giustamente: la sua guardia vede la seriale
# occupata) e resteresti senza plancia E senza mouse. Visto succedere il 15/07.
# Qui il modo mouse e' fermo -- l'abbiamo stoppato noi sopra -- quindi si puo'
# fare piazza pulita dei joypad_node senza rischio di ammazzare il suo.
# I pattern usano [x] per non matchare la riga di comando di questo script stesso.
pkill -f 'lib/joypad_controller/[j]oypad_node' 2>/dev/null
pkill -f 'lib/joypad_controller/[j]oypad_gui_app' 2>/dev/null

# Plancia chiusa: il telecomando torna a fare da mouse.
[ -x "$HOME/mouse-mode.sh" ] && "$HOME/mouse-mode.sh" start
