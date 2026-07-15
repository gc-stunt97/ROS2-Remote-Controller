#!/bin/bash
# Gestisce il "modo mouse" del telecomando: lo stick destro pilota il cursore di
# Ubuntu (vedi joypad_controller/cursor_node.py).
#
#   mouse-mode.sh start     avvia, se non gira gia'
#   mouse-mode.sh stop      ferma e ASPETTA che la seriale sia libera
#   mouse-mode.sh status
#
# PERCHE' start/stop e non "lancialo e basta":
# la seriale /dev/aira_controller deve avere UN SOLO lettore. Linux non lo
# impedisce affatto: due processi possono aprirla insieme senza il minimo errore,
# e semplicemente **si rubano i byte a vicenda** -- una riga JSON va a uno, la
# successiva all'altro, molte si spezzano a meta'. Entrambi ricevono spazzatura,
# entrambi la scartano (il joy_node butta via il JSON malformato, e fa bene), e il
# risultato e' che non funziona ne' il mouse ne' la plancia. Nessuno dei due da'
# errore: sembra che gli stick siano morti. Verificato sul campo il 15/07.
#
# Percio' gli script delle plance chiamano "stop" prima di partire e "start" alla
# chiusura: il testimone passa, e chi ha la seriale ce l'ha da solo.
#
# Installazione: vedi desktop/README.md (va copiato in ~/ come gli altri).

set -u

SERIAL="/dev/aira_controller"
LOG="/tmp/mouse-mode.log"

# I pattern usano [x] per non matchare SE STESSI: pkill -f guarda la riga di
# comando di ogni processo, compresa quella di questo script. Senza il trucco,
# "mouse-mode.sh stop" si suicida a meta' lavoro invece di fermare il nodo.
PAT_LAUNCH='[m]ouse[.]launch[.]py'
PAT_CURSOR='__node:=[c]ursor_node'
PAT_JOY='__node:=[j]oystick_node_mouse'   # nome apposta diverso da quello delle plance

serial_holders() {
    # Chi tiene aperta la seriale.
    #
    # ⚠️ Usa fuser: la scansione di /proc a mano (il fallback qui sotto) su questo
    #    Pi costa ~6 SECONDI -- 244 processi per tutti i loro fd, letti da bash uno
    #    alla volta -- e questa funzione viene chiamata in un ciclo. Era la causa
    #    degli ~11 s che il modo mouse ci metteva a tornare dopo una plancia.
    #    fuser fa lo stesso lavoro in 0,13 s: 50 volte piu' veloce, stesso risultato.
    local real
    real="$(readlink -f "$SERIAL" 2>/dev/null)" || return 1
    [ -n "$real" ] || return 1

    if command -v fuser >/dev/null 2>&1; then
        fuser "$real" 2>/dev/null | tr -s ' ' '\n' | grep -v '^$' | sort -u
        return 0
    fi

    # Fallback senza dipendenze, se un domani fuser non ci fosse: lento ma sempre
    # meglio che non sapere chi tiene la porta.
    local fd target pid
    for fd in /proc/[0-9]*/fd/*; do
        target="$(readlink -f "$fd" 2>/dev/null)" || continue
        if [ "$target" = "$real" ]; then
            pid="$(echo "$fd" | cut -d/ -f3)"
            echo "$pid"
        fi
    done | sort -u
}

is_running() {
    pgrep -f "$PAT_CURSOR" >/dev/null 2>&1
}

start() {
    if is_running; then
        echo "modo mouse: gia' attivo"
        return 0
    fi
    # Un po' di pazienza prima di rinunciare: quando una plancia si chiude i suoi
    # nodi ci mettono un attimo a mollare la seriale, e senza attesa lo start
    # fallirebbe per una frazione di secondo di ritardo. Se invece la seriale e'
    # occupata sul serio (plancia aperta), questi 3 s non cambiano l'esito.
    local holders i
    for i in $(seq 1 15); do
        holders="$(serial_holders)"
        [ -z "$holders" ] && break
        sleep 0.2
    done
    if [ -n "$holders" ]; then
        echo "modo mouse: NON avvio, la seriale e' gia' di qualcuno (PID: $(echo $holders | tr '\n' ' '))"
        echo "            probabilmente c'e' una plancia aperta: e' giusto cosi'."
        return 1
    fi
    # ⚠️ set +u attorno al source: gli script setup.bash di ROS leggono variabili
    #    non ancora definite (AMENT_TRACE_SETUP_FILES & co.), quindi con "set -u"
    #    attivo esplodono e lo script muore prima di lanciare qualsiasi cosa.
    set +u
    source /opt/ros/humble/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
    set -u
    # setsid: lo stacca dal terminale, cosi' sopravvive alla chiusura della shell
    # o della sessione SSH da cui e' stato lanciato.
    setsid nohup ros2 launch joypad_controller mouse.launch.py >"$LOG" 2>&1 </dev/null &
    echo "modo mouse: avviato (log: $LOG)"
}

stop() {
    pkill -f "$PAT_LAUNCH" 2>/dev/null
    pkill -f "$PAT_CURSOR" 2>/dev/null
    # ⚠️ Il joypad_node va ammazzato ESPLICITAMENTE: uccidendo il launch, il figlio
    #    resta orfano e continua a tenere la seriale. Visto succedere il 15/07: la
    #    plancia restava muta e sembrava un bug della plancia.
    pkill -f "$PAT_JOY" 2>/dev/null

    # Aspetta che la seriale sia davvero libera: partire prima che l'altro l'abbia
    # mollata rimette in piedi lo stesso identico problema.
    local i
    for i in $(seq 1 15); do   # ~3 s con le buone
        if [ -z "$(serial_holders)" ]; then
            echo "modo mouse: fermo, seriale libera"
            return 0
        fi
        sleep 0.2
    done

    # ⚠️ SIGTERM non basta sempre: joypad_gui_app (Tk + rclpy) e a volte il launch
    #    lo ignorano e restano appesi CON LA SERIALE IN MANO. Visto il 15/07:
    #    zombie sopravvissuti a due SIGTERM di fila. Qui la gentilezza e' finita.
    pkill -9 -f "$PAT_LAUNCH" 2>/dev/null
    pkill -9 -f "$PAT_CURSOR" 2>/dev/null
    pkill -9 -f "$PAT_JOY" 2>/dev/null
    for i in $(seq 1 15); do
        if [ -z "$(serial_holders)" ]; then
            echo "modo mouse: fermo (ci e' voluto SIGKILL), seriale libera"
            return 0
        fi
        sleep 0.2
    done
    echo "modo mouse: ATTENZIONE, la seriale e' ancora occupata da: $(serial_holders | tr '\n' ' ')"
    return 1
}

status() {
    if is_running; then
        echo "modo mouse: ATTIVO"
    else
        echo "modo mouse: fermo"
    fi
    local holders
    holders="$(serial_holders)"
    if [ -n "$holders" ]; then
        echo "seriale $SERIAL tenuta da:"
        local pid
        for pid in $holders; do
            echo "  PID $pid: $(cat "/proc/$pid/comm" 2>/dev/null)"
        done
    else
        echo "seriale $SERIAL: libera"
    fi
}

case "${1:-}" in
    start)   start ;;
    stop)    stop ;;
    restart) stop; start ;;
    status)  status ;;
    *)       echo "uso: $(basename "$0") {start|stop|restart|status}"; exit 2 ;;
esac
