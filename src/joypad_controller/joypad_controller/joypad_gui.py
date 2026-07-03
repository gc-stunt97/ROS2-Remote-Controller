#!/usr/bin/env python3
"""GUI dei joystick + PLANCIA di controllo robot — "plancia 2D".

A sinistra: i due joystick come pad quadrati con pallino che segue X/Y, crosshair,
barra Z (yaw), valori numerici. Il pallino e' vuoto a riposo, pieno col tastino.

A destra la plancia:
- "FINESTRE": bottoni toggle per avviare/chiudere RViz e il Video (sottoprocessi),
  cosi' apri/chiudi ciascuno quando vuoi senza terminale.
- "CONTROLLO ROBOT" (touch-friendly), che sostituisce i comandi `ros2 param set`.
  Setta i parametri dei nodi sul ROBOT via il parameter service (WiFi/DDS):
  - Modalita' (Manuale/Gait)  -> /teleop  left_stick_mode
  - Pattern (tripod/ripple/wave) -> /teleop gait_pattern
  - Gamba (FL..RR, ALL)       -> /teleop  selected_leg
  - Slider stride/period/duty/stance_up -> /teleop
  - Toggle SIM <-> REAL       -> /servo_node enabled  (con conferma prima del REAL)

Sottoscrive: left/right_joystick_data (Point), left/right_button (Bool).
Integrazione ROS+Tk: il ciclo lo guida Tkinter (mainloop); a ogni tick spin_once.
"""

import math
import os
import signal
import socket
import subprocess
import tkinter as tk
from tkinter import messagebox

import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

# --- estetica (catppuccin) ---
PAD = 165        # lato del pad quadrato (px) — contenuto entro i 600px del 7"
DOT_R = 8        # raggio del pallino
BAR_W = 26       # larghezza della barra Z
REFRESH_MS = 30  # ~33 Hz di ridisegno

BG = "#1e1e2e"
FG = "#cdd6f4"
GRID = "#45475a"
BTN = "#313244"        # bottone a riposo
SEL = "#89b4fa"        # selezionato (blu)
INK = "#11111b"        # testo su sfondo chiaro
DOT_LEFT = "#89b4fa"   # blu (joystick sinistro)
DOT_RIGHT = "#f38ba8"  # rosa (joystick destro)
BAR_COL = "#a6e3a1"    # verde (barra Z)
SIM_COL = "#a6e3a1"    # verde (SIM)
REAL_COL = "#f38ba8"   # rosso (REAL)

# Percorsi delle finestre avviabili dalla plancia (override con variabili d'ambiente).
RVIZ_LAUNCH = os.path.expanduser(
    os.environ.get("ROBOTHEX_RVIZ_LAUNCH", "~/ros2_ws/viz/display.launch.py"))
VIDEO_SCRIPT = os.path.expanduser(
    os.environ.get("ROBOTHEX_VIDEO_RECEIVER", "~/ros2_ws/camera/stream_receiver.sh"))
VIDEO_PORT = os.environ.get("ROBOTHEX_VIDEO_PORT", "5000")


class ProcButton:
    """Bottone toggle che avvia/chiude un sottoprocesso (RViz, video)."""

    def __init__(self, parent, name, cmd, check_path=None, on_color=SIM_COL,
                 on_start=None, on_stop=None):
        self.name = name
        self.cmd = cmd
        self.check_path = check_path
        self.on_color = on_color
        self.on_start = on_start   # azione extra all'avvio (es. accendi il sender sul robot)
        self.on_stop = on_stop     # azione extra alla chiusura
        self.proc = None
        self.btn = tk.Button(parent, text=f"Avvia {name}", bg=BTN, fg=FG,
                             relief=tk.FLAT, width=12, command=self.toggle)

    def pack(self, **kw):
        self.btn.pack(**kw)
        return self

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def toggle(self):
        self.stop() if self.running() else self.start()

    def start(self):
        if self.check_path and not os.path.exists(self.check_path):
            messagebox.showerror("File mancante",
                                 f"Non trovo:\n{self.check_path}\nHai fatto git pull?")
            return
        try:
            self.proc = subprocess.Popen(
                self.cmd, preexec_fn=os.setsid,
                env={**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":0")})
        except Exception as exc:                       # noqa: BLE001
            messagebox.showerror("Errore avvio", f"{self.name}: {exc}")
            self.proc = None
        if self.proc is not None and self.on_start:
            self.on_start()
        self.refresh()

    def stop(self):
        if self.proc is not None:
            try:                                       # SIGINT al gruppo = chiusura pulita
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
            self.proc = None
        if self.on_stop:
            self.on_stop()
        self.refresh()

    def refresh(self):
        """Aggiorna l'aspetto (rileva anche la chiusura esterna della finestra)."""
        if self.running():
            self.btn.configure(text=f"Chiudi {self.name}", bg=self.on_color, fg=INK)
        else:
            self.btn.configure(text=f"Avvia {self.name}", bg=BTN, fg=FG)


class JoypadGui:
    """Finestra Tkinter: visualizza i due joystick + plancia di controllo robot."""

    def __init__(self, node):
        self.node = node
        self.values = {"left": Point(), "right": Point()}
        self.buttons = {"left": False, "right": False}
        self._real = False

        node.create_subscription(
            Point, "left_joystick_data", lambda m: self._store("left", m), 10)
        node.create_subscription(
            Point, "right_joystick_data", lambda m: self._store("right", m), 10)
        node.create_subscription(
            Bool, "left_button", lambda m: self._store_btn("left", m), 10)
        node.create_subscription(
            Bool, "right_button", lambda m: self._store_btn("right", m), 10)

        # client dei parameter service dei nodi sul robot
        self._param_clients = {
            "teleop": node.create_client(SetParameters, "/teleop/set_parameters"),
            "servo_node": node.create_client(SetParameters, "/servo_node/set_parameters"),
            "camera_manager": node.create_client(SetParameters, "/camera_manager/set_parameters"),
        }

        self.root = tk.Tk()
        self.root.title("RobotHex — controller")
        self.root.configure(bg=BG)

        # Layout ORIZZONTALE: joystick a sinistra, plancia di controllo a destra.
        # Cosi' l'altezza resta bassa e ci sta tutto sui 600px del display 7".
        main = tk.Frame(self.root, bg=BG)
        main.pack(side=tk.TOP, padx=10, pady=6)
        self.widgets = {}

        # Colonna sinistra: i due joystick + (sotto, nello spazio libero) il toggle SIM/REAL,
        # cosi' il comando di sicurezza e' SEMPRE visibile anche su schermi bassi (800x480).
        left = tk.Frame(main, bg=BG)
        left.pack(side=tk.LEFT, anchor="n")
        joys = tk.Frame(left, bg=BG)
        joys.pack(side=tk.TOP)
        self._build_side(joys, "LEFT", "left", DOT_LEFT, tk.LEFT)
        self._build_side(joys, "RIGHT", "right", DOT_RIGHT, tk.LEFT)
        self._build_simreal(left)
        # Modalita' e Pattern stanno QUI (colonna sinistra, larga come i due joystick):
        # cosi' i 3 bottoni del Pattern (tripod/ripple/wave) ci stanno tutti senza uscire
        # dallo schermo, cosa che succedeva nel pannello destro (piu' stretto).
        self._build_modepattern(left)

        panel = tk.Frame(main, bg=BG)
        panel.pack(side=tk.LEFT, anchor="n", padx=(16, 0))
        self._build_panel(panel)

        # alla chiusura della plancia, chiudi anche RViz/Video eventualmente aperti
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # --- invio parametri ai nodi del robot --------------------------------
    @staticmethod
    def _make_param(name, value):
        p = Parameter()
        p.name = name
        v = ParameterValue()
        if isinstance(value, bool):            # bool PRIMA di int (bool e' sottotipo di int)
            v.type = ParameterType.PARAMETER_BOOL
            v.bool_value = value
        elif isinstance(value, int):
            v.type = ParameterType.PARAMETER_INTEGER
            v.integer_value = value
        elif isinstance(value, float):
            v.type = ParameterType.PARAMETER_DOUBLE
            v.double_value = value
        else:
            v.type = ParameterType.PARAMETER_STRING
            v.string_value = str(value)
        p.value = v
        return p

    def _set_param(self, target, name, value):
        cli = self._param_clients[target]
        if not cli.service_is_ready():
            self.node.get_logger().warn(
                f"/{target} non raggiungibile: il nodo gira sul robot? "
                f"({name}={value} non inviato)")
            return
        req = SetParameters.Request()
        req.parameters = [self._make_param(name, value)]
        cli.call_async(req)   # fire-and-forget: la risposta la smaltisce spin_once

    # --- costruzione UI: joystick -----------------------------------------
    def _build_side(self, parent, title, key, dot_color, side):
        frame = tk.Frame(parent, bg=BG)
        frame.pack(side=side, padx=20, pady=(12, 6))

        tk.Label(frame, text=title, bg=BG, fg=FG,
                 font=("TkDefaultFont", 13, "bold")).pack(pady=(0, 6))

        row = tk.Frame(frame, bg=BG)
        row.pack()
        pad = tk.Canvas(row, width=PAD, height=PAD, bg=BG,
                        highlightthickness=1, highlightbackground=GRID)
        pad.pack(side=tk.LEFT)
        bar = tk.Canvas(row, width=BAR_W, height=PAD, bg=BG,
                        highlightthickness=1, highlightbackground=GRID)
        bar.pack(side=tk.LEFT, padx=(8, 0))

        val = tk.Label(frame, text="X:+0.00  Y:+0.00  Z:+0.00", bg=BG, fg=FG,
                       font=("TkFixedFont", 10))
        val.pack(pady=(6, 0))

        self.widgets[key] = {"pad": pad, "bar": bar, "val": val, "color": dot_color}

    # --- costruzione UI: plancia di controllo -----------------------------
    def _build_panel(self, parent):
        p = tk.Frame(parent, bg=BG)
        p.pack(side=tk.TOP, fill=tk.X, padx=20, pady=(2, 4))

        # IP di questo controller (utile per SSH / diagnosi rete). Letto all'avvio: in DHCP
        # cambia di rado e leggerlo a ogni frame aprirebbe un socket 33 volte al secondo.
        self._ip_lbl = tk.Label(p, text=f"IP controller: {self._my_ip() or '?'}",
                                 bg=BG, fg=GRID, font=("TkFixedFont", 9))
        self._ip_lbl.pack(anchor="w", pady=(0, 6))

        tk.Label(p, text="FINESTRE", bg=BG, fg=FG,
                 font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 4))
        wins = tk.Frame(p, bg=BG)
        wins.pack(anchor="w", pady=(0, 10))
        self._procs = [
            ProcButton(wins, "RViz", ["ros2", "launch", RVIZ_LAUNCH, "gui:=false"],
                       check_path=RVIZ_LAUNCH).pack(side=tk.LEFT, padx=(0, 6)),
            ProcButton(wins, "Video", ["bash", VIDEO_SCRIPT, VIDEO_PORT],
                       check_path=VIDEO_SCRIPT,
                       on_start=self._video_on, on_stop=self._video_off).pack(side=tk.LEFT),
        ]
        # Toggle "plancia sempre in primo piano": se RViz va fullscreen la plancia resta
        # raggiungibile -> "Chiudi RViz" senza tastiera. ON=blu.
        self._topmost = False
        self._topmost_btn = tk.Button(wins, text="Sopra", bg=BTN, fg=FG, relief=tk.FLAT,
                                      width=7, command=self._toggle_topmost)
        self._topmost_btn.pack(side=tk.LEFT, padx=(6, 0))

        # Gambe (modalita' Manuale): SPUNTE multiple -> se ne muovono piu' di una insieme.
        # Disposte come la vista dall'alto del robot (FL FR / ML MR / RL RR) per intuitivita'.
        # Griglia e bottoni Tutte/Nessuna AFFIANCATI per tenere bassa l'altezza (schermo 7").
        tk.Label(p, text="CONTROLLO ROBOT — Gambe (Manuale):", bg=BG, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w", pady=(2, 2))
        legbox = tk.Frame(p, bg=BG)
        legbox.pack(anchor="w", pady=(0, 2))
        grid = tk.Frame(legbox, bg=BG)
        grid.pack(side=tk.LEFT)
        self._leg_vars = {}
        for r, rowlegs in enumerate([("FL", "FR"), ("ML", "MR"), ("RL", "RR")]):
            for c, leg in enumerate(rowlegs):
                var = tk.BooleanVar(value=False)   # nessuna gamba spuntata all'avvio
                cb = tk.Checkbutton(grid, text=leg, variable=var, command=self._on_legs,
                                    bg=BG, fg=FG, selectcolor=BTN, activebackground=BG,
                                    activeforeground=FG, width=4, anchor="w",
                                    highlightthickness=0)
                cb.grid(row=r, column=c, padx=2, pady=0, sticky="w")
                self._leg_vars[leg] = var
        qcol = tk.Frame(legbox, bg=BG)
        qcol.pack(side=tk.LEFT, padx=(12, 0))
        tk.Button(qcol, text="Tutte", bg=BTN, fg=FG, relief=tk.FLAT, width=7,
                  command=lambda: self._set_all_legs(True)).pack(fill=tk.X, pady=1)
        tk.Button(qcol, text="Nessuna", bg=BTN, fg=FG, relief=tk.FLAT, width=7,
                  command=lambda: self._set_all_legs(False)).pack(fill=tk.X, pady=1)

        # stance_up: comune a TUTTE le modalita' (altezza corpo = punto di partenza di
        # manuale/gait/corpo). Corpo piu' alto = piu' negativo. Limite fisico -140 (gamba
        # L=140 mm: oltre, l'IK va fuori portata e la zampa non risponde).
        self._slider(p, "stance_up (mm)", -140, -60, -100, "teleop", "stance_up")

        # --- slider CONTESTUALI: compare solo il gruppo della Modalita' attiva (spazio 7") ---
        self._mode_frames = {}
        fg = tk.Frame(p, bg=BG)
        self._mode_frames["gait"] = fg
        self._slider(fg, "stride (mm)", 0, 120, 60, "teleop", "stride")
        self._slider(fg, "period (s)", 0.5, 4.0, 2.0, "teleop", "period", res=0.1)
        self._slider(fg, "duty", 0.3, 0.9, 0.5, "teleop", "duty", res=0.05)

        fm = tk.Frame(p, bg=BG)
        self._mode_frames["leg_manual"] = fm
        self._slider(fm, "swing sens (°)", 10, 90, 90, "teleop", "swing_range",
                     res=5, conv=math.radians)
        self._slider(fm, "lift sens (°)", 10, 90, 90, "teleop", "lift_range",
                     res=5, conv=math.radians)

        fb = tk.Frame(p, bg=BG)
        self._mode_frames["body"] = fb
        self._slider(fb, "roll (°)", 0, 25, 11, "teleop", "body_roll_range",
                     res=1, conv=math.radians)
        self._slider(fb, "pitch (°)", 0, 25, 11, "teleop", "body_pitch_range",
                     res=1, conv=math.radians)
        self._slider(fb, "yaw (°)", 0, 35, 17, "teleop", "body_yaw_range",
                     res=1, conv=math.radians)

        self._show_mode_sliders("leg_manual")   # vista iniziale = default Modalita'

    def _build_simreal(self, parent):
        """Toggle SIM/REAL (comando di sicurezza) — sotto i joystick, sempre visibile."""
        f = tk.Frame(parent, bg=BG)
        f.pack(side=tk.TOP, fill=tk.X, pady=(12, 0), padx=4)
        # height=1 (piu' basso di prima) per lasciare spazio a Modalita'/Pattern e a
        # controlli futuri; resta comunque ben visibile perche' occupa tutta la larghezza.
        self._simreal = tk.Button(f, text="SIM  —  servi spenti", bg=SIM_COL, fg=INK,
                                   font=("TkDefaultFont", 13, "bold"), relief=tk.FLAT,
                                   height=1, command=self._toggle_simreal)
        self._simreal.pack(fill=tk.X)

    def _build_modepattern(self, parent):
        """Modalita' (Manuale/Gait) e Pattern (tripod/ripple/wave) sotto il toggle SIM/REAL."""
        f = tk.Frame(parent, bg=BG)
        f.pack(side=tk.TOP, fill=tk.X, pady=(10, 0), padx=4)
        self._segmented(f, "Modalita'",
                        [("Manuale", "leg_manual"), ("Gait", "gait"), ("Corpo", "body")],
                        "leg_manual", self._on_mode)
        self._segmented(f, "Pattern",
                        [("tripod", "tripod"), ("ripple", "ripple"), ("wave", "wave")],
                        "ripple",
                        lambda v: self._set_param("teleop", "gait_pattern", v))

    def _segmented(self, parent, label, options, default, on):
        row = tk.Frame(parent, bg=BG)
        row.pack(anchor="w", pady=3)
        tk.Label(row, text=label + ":", bg=BG, fg=FG, width=10, anchor="w").pack(side=tk.LEFT)
        btns = {}

        def highlight(val):
            for v, b in btns.items():
                b.configure(bg=SEL if v == val else BTN, fg=INK if v == val else FG)

        def select(val):
            highlight(val)
            on(val)

        for text, val in options:
            b = tk.Button(row, text=text, bg=BTN, fg=FG, width=8, relief=tk.FLAT,
                          command=lambda v=val: select(v))
            b.pack(side=tk.LEFT, padx=2)
            btns[val] = b
        highlight(default)    # solo evidenza all'avvio, senza sparare il parametro
        return btns

    def _slider(self, parent, label, lo, hi, default, target, name, res=1.0, conv=None):
        # conv: trasforma il valore dello slider nel valore del parametro (es. math.radians
        # per mostrare gradi ma inviare radianti). Default: identita'.
        row = tk.Frame(parent, bg=BG)
        row.pack(anchor="w", pady=0, fill=tk.X)
        tk.Label(row, text=label, bg=BG, fg=FG, width=14, anchor="w").pack(side=tk.LEFT)
        s = tk.Scale(row, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL,
                     bg=BG, fg=FG, troughcolor=GRID, highlightthickness=0,
                     length=200, sliderrelief=tk.FLAT)
        s.set(default)        # imposta prima di cablare il command -> niente invio all'avvio
        s.configure(command=lambda v: self._set_param(
            target, name, conv(float(v)) if conv else float(v)))
        s.pack(side=tk.LEFT)
        return s

    def _toggle_topmost(self):
        """Tiene la plancia sempre in primo piano (utile se RViz va fullscreen)."""
        self._topmost = not self._topmost
        self.root.attributes("-topmost", self._topmost)
        self._topmost_btn.configure(bg=SEL if self._topmost else BTN,
                                    fg=INK if self._topmost else FG)

    def _on_mode(self, v):
        """Cambio Modalita': invia il parametro e mostra gli slider di quella modalita'."""
        self._set_param("teleop", "left_stick_mode", v)
        self._show_mode_sliders(v)

    def _show_mode_sliders(self, mode):
        """Mostra solo il gruppo di slider della modalita' attiva (gli altri li nasconde)."""
        for m, fr in self._mode_frames.items():
            if m == mode:
                fr.pack(anchor="w", fill=tk.X)
            else:
                fr.pack_forget()

    def _on_legs(self):
        """Spunte gambe -> parametro selected_leg del teleop (CSV, o 'ALL' se tutte)."""
        sel = [leg for leg, v in self._leg_vars.items() if v.get()]
        if len(sel) == len(self._leg_vars):
            value = "ALL"                 # tutte spuntate -> piu' compatto
        else:
            value = ",".join(sel)         # es. 'FL,MR,RR'; nessuna -> "" (robot non muove)
        self._set_param("teleop", "selected_leg", value)

    def _set_all_legs(self, on):
        for v in self._leg_vars.values():
            v.set(on)
        self._on_legs()

    def _toggle_simreal(self):
        if not self._real:   # sto per attivare i servi VERI -> conferma di sicurezza
            if not messagebox.askyesno(
                    "Attiva REAL",
                    "Attivare i SERVI VERI?\nIl robot si muovera' secondo il comando corrente."):
                return
        self._real = not self._real
        self._set_param("servo_node", "enabled", self._real)
        if self._real:
            self._simreal.configure(text="REAL  —  servi ATTIVI", bg=REAL_COL)
        else:
            self._simreal.configure(text="SIM  —  servi spenti", bg=SIM_COL)

    def _my_ip(self):
        """IP di questo controller (per dire al robot dove mandare il video)."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))   # non invia nulla: sceglie l'interfaccia di uscita
            return s.getsockname()[0]
        except OSError:
            return None
        finally:
            s.close()

    def _video_on(self):
        """Avvio video: dico al camera_manager sul robot di partire, verso il MIO IP."""
        ip = self._my_ip()
        if ip:
            self._set_param("camera_manager", "host", ip)
        self._set_param("camera_manager", "enabled", True)

    def _video_off(self):
        self._set_param("camera_manager", "enabled", False)

    # --- callback ROS -----------------------------------------------------
    def _store(self, key, msg):
        self.values[key] = msg

    def _store_btn(self, key, msg):
        self.buttons[key] = bool(msg.data)

    @staticmethod
    def _clamp(v):
        return max(-1.0, min(1.0, float(v)))

    # --- disegno ----------------------------------------------------------
    def _draw_pad(self, canvas, x, y, color, pressed):
        canvas.delete("all")
        c = PAD / 2
        half = PAD / 2 - DOT_R - 2
        canvas.create_line(c, 4, c, PAD - 4, fill=GRID)
        canvas.create_line(4, c, PAD - 4, c, fill=GRID)
        canvas.create_oval(c - half, c - half, c + half, c + half, outline=GRID)
        px = c + self._clamp(x) * half           # X: destra = +
        py = c - self._clamp(y) * half           # Y: su = + (schermo invertito)
        if pressed:
            canvas.create_oval(px - DOT_R, py - DOT_R, px + DOT_R, py + DOT_R,
                               fill=color, outline="")
        else:
            canvas.create_oval(px - DOT_R, py - DOT_R, px + DOT_R, py + DOT_R,
                               outline=color, width=2)

    def _draw_bar(self, canvas, z):
        canvas.delete("all")
        c = PAD / 2
        top = c - self._clamp(z) * (PAD / 2 - 4)
        canvas.create_line(2, c, BAR_W - 2, c, fill=GRID)
        canvas.create_rectangle(4, min(c, top), BAR_W - 4, max(c, top),
                                fill=BAR_COL, outline="")

    # --- ciclo principale -------------------------------------------------
    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for key, w in self.widgets.items():
            m = self.values[key]
            self._draw_pad(w["pad"], m.x, m.y, w["color"], self.buttons[key])
            self._draw_bar(w["bar"], m.z)
            w["val"].config(text=f"X:{m.x:+.2f}  Y:{m.y:+.2f}  Z:{m.z:+.2f}")
        for pb in self._procs:      # aggiorna i bottoni RViz/Video (anche se chiusi a mano)
            pb.refresh()
        self.root.after(REFRESH_MS, self._tick)

    def _on_close(self):
        for pb in getattr(self, "_procs", []):
            pb.stop()
        self.root.destroy()

    def run(self):
        self.root.after(0, self._tick)
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("joystick_gui")
    gui = JoypadGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
