#!/bin/sh
# Condivide lo schermo del telecomando (display :0) via VNC, porta 5900, con password.
# Dal PC: col cavo sullo switch del robot -> VNC su 192.168.10.10; in WiFi -> tunnel SSH
# (ssh -L 5901:localhost:5900 giulio@Controller) e VNC su localhost::5901.
# Parte da solo al login (~/.config/autostart/AIRA-VNC.desktop). Log in ~/.vnc/x11vnc.log
command -v x11vnc >/dev/null || { echo "manca x11vnc: sudo apt install x11vnc"; exit 1; }
[ -f "$HOME/.vnc/passwd" ] || { echo "manca la password: x11vnc -storepasswd"; exit 1; }
pkill -u "$USER" -x x11vnc 2>/dev/null
# -forever: resta su dopo che il PC si stacca; -shared: piu' visualizzatori insieme;
# -noxdamage: niente pezzi di schermo vecchi; -repeat: tieni l'autorepeat della tastiera
# -scale 2:nb: al PC arriva 1600x960 (lo schermo e' 800x480), pixel raddoppiati senza sfocare
# -allow: solo dalla rete del robot (RUT241) e dal tunnel SSH (127.0.0.1). In WiFi la password VNC
#   (8 caratteri, traffico in chiaro) non basta: si entra col tunnel, vedi desktop/README.md
exec x11vnc -display :0 -forever -shared -rfbauth "$HOME/.vnc/passwd" -rfbport 5900 \
    -scale 2:nb -allow 192.168.10.,127.0.0.1 -noxdamage -repeat -quiet -o "$HOME/.vnc/x11vnc.log"
