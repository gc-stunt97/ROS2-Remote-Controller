#!/usr/bin/env python3
"""Dashboard di guida della base mobile AIRA (plancia 2D dedicata).

Gemella della plancia di RobotHex (`joypad_controller/joypad_gui.py`) ma per una BASE
MOBILE: stessi pattern (Tk+rclpy, pad joystick, ProcButton, IP, always-on-top, chiamate
ai nodi del robot via WiFi/DDS), contenuti diversi.

A sinistra: i due pad joystick (la base si guida con lo stick SINISTRO: y=avanti, x=sterzo)
  + il toggle di sicurezza MOTORI ON/OFF (chiama /odrive_node/enable_motors, con conferma).
A destra:
  - TELEMETRIA ODrive (da /odrive_status, diagnostic_msgs/DiagnosticArray):
    Vbus, e per asse stato/corrente/temp/errori (verde=ok, rosso=errore, grigio=offline).
  - VELOCITA': comandata (/cmd_vel) vs reale (/odom), a barre.
  - MINI-MAPPA: posa da /odom (x,y,theta) con scia del percorso (robot al centro).
  - FINESTRE: bottone per aprire/chiudere RViz (modello + odom sul display del controller).
  - IP del controller + toggle "Sopra" (always-on-top).

Sottoscrive: left/right_joystick_data (Point), left/right_button (Bool),
  cmd_vel (Twist), odom (Odometry), odrive_status (DiagnosticArray).
Integrazione ROS+Tk: il ciclo lo guida Tkinter (mainloop); a ogni tick spin_once.
"""

import math
import os
import signal
import socket
import subprocess
from collections import deque
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# --- estetica (catppuccin, come la plancia RobotHex) ---
PAD = 150
DOT_R = 8
BAR_W = 24
REFRESH_MS = 33  # ~30 Hz

BG = "#1e1e2e"
FG = "#cdd6f4"
GRID = "#45475a"
BTN = "#313244"
SEL = "#89b4fa"
INK = "#11111b"
DOT_LEFT = "#89b4fa"
DOT_RIGHT = "#f38ba8"
BAR_COL = "#a6e3a1"
OK_COL = "#a6e3a1"     # verde
WARN_COL = "#f9e2af"   # giallo
ERR_COL = "#f38ba8"    # rosso
OFF_COL = "#6c7086"    # grigio (offline/stale)
CMD_COL = "#89b4fa"    # blu (comandato)
REAL_COL = "#a6e3a1"   # verde (reale)
TRAIL_COL = "#585b70"

# servizio del driver AIRA per accendere/spegnere i motori (closed-loop <-> idle)
ENABLE_SRV = os.environ.get("AIRA_ENABLE_SRV", "/odrive_node/enable_motors")
# launch RViz per AIRA (sul controller): modello + odom. Override con variabile d'ambiente.
RVIZ_LAUNCH = os.path.expanduser(
    os.environ.get("AIRA_RVIZ_LAUNCH", "~/ros2_ws/viz/aira/display.launch.py"))

DIAG_LEVEL_COLOR = {
    DiagnosticStatus.OK: OK_COL,
    DiagnosticStatus.WARN: WARN_COL,
    DiagnosticStatus.ERROR: ERR_COL,
    DiagnosticStatus.STALE: OFF_COL,
}

AXIS_STATE_CLOSED_LOOP = 8   # enum ODrive: asse in coppia (closed-loop control)


class ProcButton:
    """Bottone toggle che avvia/chiude un sottoprocesso (es. RViz). Come nella plancia."""

    def __init__(self, parent, name, cmd, check_path=None, on_color=OK_COL):
        self.name = name
        self.cmd = cmd
        self.check_path = check_path
        self.on_color = on_color
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
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Errore avvio", f"{self.name}: {exc}")
            self.proc = None
        self.refresh()

    def stop(self):
        if self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
            self.proc = None
        self.refresh()

    def refresh(self):
        if self.running():
            self.btn.configure(text=f"Chiudi {self.name}", bg=self.on_color, fg=INK)
        else:
            self.btn.configure(text=f"Avvia {self.name}", bg=BTN, fg=FG)


class AiraDashboard:
    def __init__(self, node):
        self.node = node
        self.values = {"left": Point(), "right": Point()}
        self.buttons = {"left": False, "right": False}
        self.cmd = Twist()
        self.odom = Odometry()
        self.status = {}          # name -> DiagnosticStatus (ultimo /odrive_status)
        self._motors_on = False   # stato REALE dei motori, dedotto dalla telemetria
        self._trail = deque(maxlen=400)   # scia (x, y) per la mini-mappa

        node.create_subscription(Point, "left_joystick_data",
                                 lambda m: self._store("left", m), 10)
        node.create_subscription(Point, "right_joystick_data",
                                 lambda m: self._store("right", m), 10)
        node.create_subscription(Bool, "left_button",
                                 lambda m: self._store_btn("left", m), 10)
        node.create_subscription(Bool, "right_button",
                                 lambda m: self._store_btn("right", m), 10)
        node.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        node.create_subscription(Odometry, "odom", self._on_odom, 10)
        node.create_subscription(DiagnosticArray, "odrive_status", self._on_status, 10)

        self._enable_cli = node.create_client(SetBool, ENABLE_SRV)

        # --- TESTA: il pulsante touch pubblica /head/enable. Latched (transient_local)
        # cosi' il bridge sul robot riceve l'ultimo stato anche se parte dopo la plancia.
        # Il bridge/Nano lo interpretano come: ON = testa segue lo stick DX + livella;
        # OFF = resta livellata a terra col pan centrato (fail-safe di default).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._head_enabled = False
        self._head_tick = 0
        self._head_pub = node.create_publisher(Bool, "/head/enable", latched)
        self._head_pub.publish(Bool(data=False))

        self.root = tk.Tk()
        self.root.title("AIRA — dashboard base mobile")
        self.root.configure(bg=BG)

        main = tk.Frame(self.root, bg=BG)
        main.pack(side=tk.TOP, padx=10, pady=6)
        self.widgets = {}

        left = tk.Frame(main, bg=BG)
        left.pack(side=tk.LEFT, anchor="n")
        joys = tk.Frame(left, bg=BG)
        joys.pack(side=tk.TOP)
        self._build_side(joys, "SINISTRO (guida)", "left", DOT_LEFT)
        self._build_side(joys, "DESTRO (testa)", "right", DOT_RIGHT)
        self._build_motor_toggle(left)
        self._build_head_toggle(left)

        panel = tk.Frame(main, bg=BG)
        panel.pack(side=tk.LEFT, anchor="n", padx=(16, 0))
        self._build_panel(panel)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # --- costruzione UI ---------------------------------------------------
    def _build_side(self, parent, title, key, dot_color):
        frame = tk.Frame(parent, bg=BG)
        frame.pack(side=tk.LEFT, padx=16, pady=(10, 6))
        tk.Label(frame, text=title, bg=BG, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(pady=(0, 6))
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

    def _build_motor_toggle(self, parent):
        f = tk.Frame(parent, bg=BG)
        f.pack(side=tk.TOP, fill=tk.X, pady=(12, 0), padx=4)
        self._motor_btn = tk.Button(
            f, text="MOTORI  —  SPENTI", bg=OFF_COL, fg=INK,
            font=("TkDefaultFont", 13, "bold"), relief=tk.FLAT, height=1,
            command=self._toggle_motors)
        self._motor_btn.pack(fill=tk.X)

    def _build_head_toggle(self, parent):
        f = tk.Frame(parent, bg=BG)
        f.pack(side=tk.TOP, fill=tk.X, pady=(8, 0), padx=4)
        self._head_btn = tk.Button(
            f, text="TESTA  —  livella (stick off)", bg=OFF_COL, fg=INK,
            font=("TkDefaultFont", 13, "bold"), relief=tk.FLAT, height=1,
            command=self._toggle_head)
        self._head_btn.pack(fill=tk.X)

    def _build_panel(self, parent):
        p = tk.Frame(parent, bg=BG)
        p.pack(side=tk.TOP, fill=tk.X)

        # riga IP + "Sopra"
        toprow = tk.Frame(p, bg=BG)
        toprow.pack(anchor="w", fill=tk.X, pady=(0, 6))
        tk.Label(toprow, text=f"IP controller: {self._my_ip() or '?'}",
                 bg=BG, fg=GRID, font=("TkFixedFont", 9)).pack(side=tk.LEFT)
        self._topmost = False
        self._topmost_btn = tk.Button(toprow, text="Sopra", bg=BTN, fg=FG, relief=tk.FLAT,
                                      width=7, command=self._toggle_topmost)
        self._topmost_btn.pack(side=tk.LEFT, padx=(12, 0))

        # --- telemetria ODrive ---
        tk.Label(p, text="TELEMETRIA ODrive", bg=BG, fg=FG,
                 font=("TkDefaultFont", 12, "bold")).pack(anchor="w")
        tele = tk.Frame(p, bg=BG)
        tele.pack(anchor="w", fill=tk.X, pady=(2, 8))
        self._vbus_lbl = tk.Label(tele, text="Vbus: -- V   ibus: -- A", bg=BG, fg=FG,
                                  font=("TkFixedFont", 11))
        self._vbus_lbl.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 2))
        self._axis_lbls = {}
        for i, axis in enumerate(("axis_left", "axis_right")):
            dot = tk.Canvas(tele, width=14, height=14, bg=BG, highlightthickness=0)
            dot.grid(row=1 + i, column=0, sticky="w", padx=(0, 4))
            oval = dot.create_oval(2, 2, 12, 12, fill=OFF_COL, outline="")
            lbl = tk.Label(tele, text=f"{axis}: offline", bg=BG, fg=FG,
                           font=("TkFixedFont", 10), anchor="w", justify="left")
            lbl.grid(row=1 + i, column=1, sticky="w")
            self._axis_lbls[axis] = (dot, oval, lbl)

        # --- velocita' comandata vs reale ---
        tk.Label(p, text="VELOCITA'  (blu=comandata, verde=reale)", bg=BG, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        velf = tk.Frame(p, bg=BG)
        velf.pack(anchor="w", pady=(2, 8))
        tk.Label(velf, text="lin", bg=BG, fg=FG, width=4, anchor="w").grid(row=0, column=0)
        self._lin_canvas = tk.Canvas(velf, width=220, height=18, bg=BG,
                                     highlightthickness=1, highlightbackground=GRID)
        self._lin_canvas.grid(row=0, column=1)
        self._lin_lbl = tk.Label(velf, text="0.00 / 0.00", bg=BG, fg=FG,
                                 font=("TkFixedFont", 9), width=13)
        self._lin_lbl.grid(row=0, column=2)
        tk.Label(velf, text="ang", bg=BG, fg=FG, width=4, anchor="w").grid(row=1, column=0)
        self._ang_canvas = tk.Canvas(velf, width=220, height=18, bg=BG,
                                     highlightthickness=1, highlightbackground=GRID)
        self._ang_canvas.grid(row=1, column=1)
        self._ang_lbl = tk.Label(velf, text="0.00 / 0.00", bg=BG, fg=FG,
                                 font=("TkFixedFont", 9), width=13)
        self._ang_lbl.grid(row=1, column=2)

        # --- mini-mappa odom ---
        mapf = tk.Frame(p, bg=BG)
        mapf.pack(anchor="w", pady=(0, 8))
        tk.Label(mapf, text="ODOM", bg=BG, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        self._map = tk.Canvas(mapf, width=190, height=190, bg=BG,
                              highlightthickness=1, highlightbackground=GRID)
        self._map.pack()
        self._pose_lbl = tk.Label(mapf, text="x:0.00 y:0.00 th:0°", bg=BG, fg=FG,
                                  font=("TkFixedFont", 9))
        self._pose_lbl.pack(anchor="w")

        # --- finestre (RViz) ---
        tk.Label(p, text="FINESTRE", bg=BG, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        wins = tk.Frame(p, bg=BG)
        wins.pack(anchor="w", pady=(2, 0))
        self._procs = [
            ProcButton(wins, "RViz", ["ros2", "launch", RVIZ_LAUNCH, "gui:=false"],
                       check_path=RVIZ_LAUNCH).pack(side=tk.LEFT),
        ]

    # --- azioni -----------------------------------------------------------
    def _toggle_motors(self):
        target = not self._motors_on   # inverti lo stato REALE letto dalla telemetria
        if target:   # sto per ATTIVARE i motori -> conferma di sicurezza
            if not messagebox.askyesno(
                    "Attiva motori",
                    "Attivare i MOTORI?\nLe ruote andranno in coppia e la base\n"
                    "si muovera' secondo il comando corrente."):
                return
        if not self._enable_cli.service_is_ready():
            messagebox.showwarning(
                "Driver non raggiungibile",
                f"Il servizio {ENABLE_SRV} non risponde.\n"
                "Il driver (odrive_node) gira sul robot ed e' in rete?")
            return
        req = SetBool.Request()
        req.data = target
        self._enable_cli.call_async(req)   # fire-and-forget; il vero stato lo conferma la telemetria

    def _render_motor_btn(self, online):
        if not online:
            self._motor_btn.configure(text="MOTORI  —  offline", bg=OFF_COL)
        elif self._motors_on:
            self._motor_btn.configure(text="MOTORI  —  ATTIVI", bg=OK_COL)
        else:
            self._motor_btn.configure(text="MOTORI  —  SPENTI", bg=WARN_COL)

    def _toggle_head(self):
        self._head_enabled = not self._head_enabled
        self._head_pub.publish(Bool(data=self._head_enabled))
        self._render_head_btn()

    def _render_head_btn(self):
        if self._head_enabled:
            self._head_btn.configure(text="TESTA  —  ATTIVA (stick dx)", bg=OK_COL)
        else:
            self._head_btn.configure(text="TESTA  —  livella (stick off)", bg=OFF_COL)

    def _toggle_topmost(self):
        self._topmost = not self._topmost
        self.root.attributes("-topmost", self._topmost)
        self._topmost_btn.configure(bg=SEL if self._topmost else BTN,
                                    fg=INK if self._topmost else FG)

    def _my_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        except OSError:
            return None
        finally:
            s.close()

    # --- callback ROS -----------------------------------------------------
    def _store(self, key, msg):
        self.values[key] = msg

    def _store_btn(self, key, msg):
        self.buttons[key] = bool(msg.data)

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        self._trail.append((p.x, p.y))

    def _on_status(self, msg):
        self.status = {s.name: s for s in msg.status}

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
        px = c + self._clamp(x) * half
        py = c - self._clamp(y) * half
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

    def _draw_vbar(self, canvas, real, cmd, vmax):
        """Barra orizzontale centrata a zero: riempimento = reale, tacca = comandato."""
        canvas.delete("all")
        w = int(canvas["width"])
        h = int(canvas["height"])
        c = w / 2
        canvas.create_line(c, 0, c, h, fill=GRID)
        vmax = max(vmax, 1e-3)
        rx = c + max(-1.0, min(1.0, real / vmax)) * (c - 2)
        canvas.create_rectangle(min(c, rx), 3, max(c, rx), h - 3,
                                fill=REAL_COL, outline="")
        cx = c + max(-1.0, min(1.0, cmd / vmax)) * (c - 2)
        canvas.create_line(cx, 0, cx, h, fill=CMD_COL, width=3)

    def _draw_map(self):
        cv = self._map
        cv.delete("all")
        w = int(cv["width"])
        half = w / 2
        span = 2.0                      # semi-lato vista in metri (robot al centro)
        scale = (half - 6) / span
        # griglia ogni 1 m
        for g in (-2, -1, 0, 1, 2):
            gp = half + g * scale
            cv.create_line(gp, 0, gp, w, fill=TRAIL_COL)
            cv.create_line(0, gp, w, gp, fill=TRAIL_COL)
        pos = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        th = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        # scia nel frame del ROBOT (robot al centro, avanti = su): ruoto ogni punto di -theta.
        ct, stt = math.cos(th), math.sin(th)
        pts = []
        for (tx, ty) in self._trail:
            dx, dy = tx - pos.x, ty - pos.y
            fwd = dx * ct + dy * stt        # componente avanti
            lft = -dx * stt + dy * ct       # componente a sinistra
            sx = half - lft * scale         # sinistra -> a sinistra sullo schermo
            sy = half - fwd * scale         # avanti -> in alto
            pts.extend((sx, sy))
        if len(pts) >= 4:
            cv.create_line(*pts, fill=BAR_COL, width=1)
        # robot al centro con freccia di orientamento (in vista body: avanti = su)
        cv.create_oval(half - 5, half - 5, half + 5, half + 5, fill=CMD_COL, outline="")
        cv.create_line(half, half, half, half - 14, fill=CMD_COL, width=2)
        self._pose_lbl.config(
            text=f"x:{pos.x:+.2f} y:{pos.y:+.2f} th:{math.degrees(th):+.0f}°")

    def _update_telemetry(self):
        board = self.status.get("odrive")
        if board is None:
            self._vbus_lbl.config(text="Vbus: -- V   (offline)")
        else:
            kv = {k.key: k.value for k in board.values}
            self._vbus_lbl.config(
                text=f"Vbus: {kv.get('vbus_voltage', '--')} V   "
                     f"ibus: {kv.get('ibus', '--')} A")
        states = []
        for axis, (dot, oval, lbl) in self._axis_lbls.items():
            st = self.status.get(axis)
            if st is None:
                dot.itemconfigure(oval, fill=OFF_COL)
                lbl.config(text=f"{axis}: offline")
                continue
            kv = {k.key: k.value for k in st.values}
            color = DIAG_LEVEL_COLOR.get(st.level, OFF_COL)
            dot.itemconfigure(oval, fill=color)
            lbl.config(text=f"{axis[5:]:5} st:{kv.get('state','?'):>2}  "
                            f"I:{kv.get('current_A','--'):>5}A  "
                            f"T:{kv.get('fet_temp_C','--')}°  {st.message}")
            try:
                states.append(int(kv.get("state", -1)))
            except ValueError:
                states.append(-1)

        # stato REALE motori: online se la board risponde; ON se ENTRAMBI gli assi in closed-loop
        online = board is not None
        self._motors_on = len(states) == 2 and all(s == AXIS_STATE_CLOSED_LOOP for s in states)
        self._render_motor_btn(online)

    # --- ciclo principale -------------------------------------------------
    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for key, w in self.widgets.items():
            m = self.values[key]
            self._draw_pad(w["pad"], m.x, m.y, w["color"], self.buttons[key])
            self._draw_bar(w["bar"], m.z)
            w["val"].config(text=f"X:{m.x:+.2f}  Y:{m.y:+.2f}  Z:{m.z:+.2f}")
        self._draw_vbar(self._lin_canvas, self.odom.twist.twist.linear.x,
                        self.cmd.linear.x, 0.6)
        self._draw_vbar(self._ang_canvas, self.odom.twist.twist.angular.z,
                        self.cmd.angular.z, 1.5)
        self._lin_lbl.config(
            text=f"{self.cmd.linear.x:+.2f}/{self.odom.twist.twist.linear.x:+.2f}")
        self._ang_lbl.config(
            text=f"{self.cmd.angular.z:+.2f}/{self.odom.twist.twist.angular.z:+.2f}")
        self._draw_map()
        self._update_telemetry()
        for pb in self._procs:
            pb.refresh()
        # ribadisci /head/enable ogni ~10 tick (~3 Hz): heartbeat lento per un bridge
        # che riparte, senza intasare il link (il latched copre i late-joiner).
        self._head_tick = (self._head_tick + 1) % 10
        if self._head_tick == 0:
            self._head_pub.publish(Bool(data=self._head_enabled))
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
    node = rclpy.create_node("aira_dashboard")
    gui = AiraDashboard(node)
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
