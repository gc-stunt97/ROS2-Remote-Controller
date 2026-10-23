#!/bin/sh
# Condivide lo schermo del telecomando (display :0) via VNC, porta 5900, con password.
# Dal PC: un visualizzatore VNC su "Controller" (WiFi) o 192.168.10.10 (cavo sullo switch).
# Parte da solo al login (~/.config/autostart/AIRA-VNC.desktop). Log in ~/.vnc/x11vnc.log
command -v x11vnc >/dev/null || { echo "manca x11vnc: sudo apt install x11vnc"; exit 1; }
[ -f "$HOME/.vnc/passwd" ] || { echo "manca la password: x11vnc -storepasswd"; exit 1; }
pkill -u "$USER" -x x11vnc 2>/dev/null
# -forever: resta su dopo che il PC si stacca; -shared: piu' visualizzatori insieme;
# -noxdamage: niente pezzi di schermo vecchi; -repeat: tieni l'autorepeat della tastiera
exec x11vnc -display :0 -forever -shared -rfbauth "$HOME/.vnc/passwd" -rfbport 5900 \
    -noxdamage -repeat -quiet -o "$HOME/.vnc/x11vnc.log"
