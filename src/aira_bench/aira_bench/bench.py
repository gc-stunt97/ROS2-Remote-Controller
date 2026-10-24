#!/usr/bin/env python3
"""BANCO dei micro AIRA: il telecomando finge di essere il mini PC (CONTROLLER_HANDBOOK §14).

Si attaccano i micro ESP32-P4-ETH alla porta "solo micro" del pannello (direttamente o tramite
lo switch del robot, al posto del mini PC) e questo programma:
  - ascolta gli annunci (hello) e riconosce ogni scheda dal MAC (tabella `periph/roles.yaml` nel
    clone di AIRA_Robot: e' il micro a presentarsi). Le schede in rete stanno in una LISTA in
    alto: con un solo micro noto si aggancia da solo, con piu' micro si sceglie quale (gli altri
    restano senza capo = disarmati);
  - si aggancia come CAPO: heartbeat 20 Hz con boss="banco", spinge la config del PROFILO del
    ruolo (`periph/<ruolo>.yaml`, la STESSA che spingera' il mini PC);
  - stick -> giunti (busto: stick DX Y -> pitch, stick DX X -> roll), oppure modo RAW: dallo
    stick DX push-pull (Y = motori insieme, X = in opposizione), dagli slider un motore
    ciascuno per mappare i versi;
  - mostra telemetria, fault, eta' dell'heartbeat, log del micro;
  - "stacca heartbeat" simula il cavo che cade: i motori si devono fermare da soli;
  - OTA: sceglie un .bin, lo serve via HTTP e dice al micro di scaricarlo;
  - calibrazione dei pot a due pose (angolo letto con la livella).

Funziona anche SENZA ROS (es. da un portatile): allora gli stick sono sostituiti dagli slider.
Con ROS: joystick da joy_node (la seriale ha UN SOLO lettore, sez. 12.3) e fungo su
/emergency_stop (reliable + transient_local, come le plance).

    ros2 launch aira_bench bench.launch.py          # sul telecomando
    python3 bench.py --repo ~/AIRA_Robot            # ovunque, senza ROS
"""

import argparse
import functools
import http.server
import json
import math
import os
import socket
import socketserver
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

try:
    from aira_bench import autocal
except ImportError:  # lanciato come script (python3 bench.py)
    import autocal

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

CMD_PORT = 47000          # micro: comandi
HELLO_PORT = 47001        # banco: annunci + telemetria + log (un socket solo)
HB_HZ = 20
OTA_HTTP_PORT = 8070
JOY_TIMEOUT = 1.0         # s senza messaggi joystick -> comandano gli slider
JOY_DRAIN = 20            # callback ROS svuotate a ogni giro di _tick (vedi li')
STICK_DZ = 0.08           # zona morta degli stick (a riposo stanno entro +-0.04)
# registri PER SESSIONE (23/10): uno per ogni avvio della plancia, col nome datato, cosi' chiudere
# e riaprire non cancella niente. Si tengono gli ultimi SESSION_KEEP di ciascun tipo
SESSION = time.strftime("%Y%m%d_%H%M%S")
TEL_DIR = os.path.expanduser("~/.aira/bench_tel")         # telemetria da armato, 50 Hz
LOG_DIR = os.path.expanduser("~/.aira/bench_log")         # copia della scheda log
TEL_LATEST = os.path.expanduser("~/.aira/bench_tel.jsonl")  # link all'ultima sessione (nome storico)
SESSION_KEEP = 20
# scheda PID (22/10): parametri di taratura regolabili da armato (il fw >= 0.2.9 non disarma
# per queste chiavi). (chiave cfg, etichetta, min, max, passo); "vel" manda vel_max E vel_banco
# None = casella vuota nella griglia (3 colonne, una riga per gruppo)
TUNE = [("pitch.kp", "pitch kp", 0.0, 0.50, 0.005), ("pitch.ki", "pitch ki", 0.0, 0.30, 0.002),
        ("pitch.kd", "pitch kd", 0.0, 0.01, 0.0005),
        ("roll.kp", "roll kp", 0.0, 0.50, 0.005), ("roll.ki", "roll ki", 0.0, 0.30, 0.002),
        ("roll.kd", "roll kd", 0.0, 0.01, 0.0005),   # 0-0.01: con 0-0.05 non si centrava un valore (24/10)
        ("pitch.u_max", "PWM max pitch", 0.05, 1.0, 0.01), ("roll.u_max", "PWM max roll", 0.05, 1.0, 0.01),
        ("duty_max", "PWM max motori", 0.10, 1.0, 0.01),
        ("vel", "vel max deg/s", 2.0, 120.0, 1.0), ("acc_max", "acc deg/s2", 10.0, 600.0, 10.0),
        ("duty_min", "PWM min stacco", 0.0, 0.30, 0.005),
        ("stall.duty", "stallo duty", 0.20, 1.0, 0.01), ("stall.ms", "stallo ms", 200.0, 5000.0, 100.0),
        ("comply", "cedevolezza 0=rigido", 0.0, 60.0, 1.0),     # fw >= 0.3.2: deg di cedimento per PWM di spinta
        ("gap.duty", "gioco duty 0=off", 0.0, 0.30, 0.005), ("gap.i_on", "gioco presa A", 0.05, 1.0, 0.01),
        ("gap.ms", "gioco max ms", 50.0, 600.0, 10.0),
        ("hold_u", "soglia carico PWM", 0.0, 0.20, 0.005),       # fw >= 0.3.2: sotto = a riposo (hold)
        # fw >= 0.3.4: da dove si stima la spinta per la cedevolezza: 0 integrale, 1 uscita PID, 2 ACS
        ("comply.src", "fonte cedev. 0-2", 0.0, 2.0, 1.0),   # etichette corte: 3 colonne in 800 px
        ("comply.amp", "ACS A a PWM max", 1.0, 15.0, 0.1)]
# scheda funzioni (23/10): movimenti automatici calcolati QUI e mandati come target a 20 Hz, cosi'
# la velocita' vera la limitano comunque vel_max/acc/PWM del micro (scheda PID)
FUNCS = (("manual", "Manuale (joystick)"), ("twirl", "Twirl"), ("osc_p", "Oscillatore pitch"),
         ("osc_r", "Oscillatore roll"), ("autocal", "Autocalibrazione"))
# slider visibili per funzione: un oscillatore ha il SUO swing e l'altro asse FERMO in un punto
FUNC_SLIDERS = {"twirl": ("period", "sw_p", "sw_r"), "osc_p": ("period", "sw_p", "pos_r"),
                "osc_r": ("period", "sw_r", "pos_p")}
FUNC_NOTES = {
    "twirl": "la velocita' vera la limitano vel max / acc / PWM della scheda PID",
    "osc_p": "oscilla il pitch col roll fermo al valore scelto\n"
             "la velocita' vera la limitano vel max / acc / PWM della scheda PID",
    "osc_r": "oscilla il roll col pitch fermo al valore scelto\n"
             "la velocita' vera la limitano vel max / acc / PWM della scheda PID",
    "autocal": "~4-5 min: fermo, gradini, inversioni, sinusoidi, (RAW) stacco.\n"
               "Risultati nel log e in ~/.aira/autocal/ (non si azzerano). STOP = disarmo",
}
FUNC_APPROACH = 10.0      # deg/s del target per arrivare al punto di partenza e tornare al centro
FUNC_SPINUP_S = 1.5       # s per portare la velocita' angolare da 0 a quella chiesta
FUNC_AMP_RATE = 10.0      # deg/s: un cambio di swing da slider arriva cosi', non a scalino
TEL_LOG_MAX = 50 * 1024 * 1024                             # ~1 MB/min: circa 50 min di prove
MICRO_GONE_S = 5.0        # s senza annunci -> la scheda in lista diventa "assente"

# estetica: come le plance (catppuccin)
BG, FG, GRID, BTN, INK = "#1e1e2e", "#cdd6f4", "#45475a", "#313244", "#11111b"
OK_COL, WARN_COL, ERR_COL, OFF_COL = "#a6e3a1", "#f9e2af", "#f38ba8", "#6c7086"
MEAS_COL, TGT_COL, SP_COL = "#a6e3a1", "#89b4fa", "#f9e2af"


# ---------------------------------------------------------------------- profili
def open_session_file(folder, prefix, ext, mode):
    """Apre <folder>/<prefix>_<SESSION>.<ext> e cancella i piu' vecchi oltre SESSION_KEEP."""
    os.makedirs(folder, exist_ok=True)
    old = sorted(f for f in os.listdir(folder) if f.startswith(prefix + "_") and f.endswith("." + ext))
    for f in old[:max(0, len(old) - (SESSION_KEEP - 1))]:
        try:
            os.remove(os.path.join(folder, f))
        except OSError:
            pass
    return open(os.path.join(folder, f"{prefix}_{SESSION}.{ext}"), mode)


def load_yaml(path):
    if yaml is None:
        raise RuntimeError("manca PyYAML: sudo apt install python3-yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class Profiles:
    """Profili dal clone di AIRA_Robot + calibrazione locale in ~/.aira/periph/<ruolo>.yaml."""

    def __init__(self, repo):
        self.repo = os.path.expanduser(repo)
        self.dir = os.path.join(self.repo, "periph")
        self.local_dir = os.path.expanduser("~/.aira/periph")

    def roles(self):
        p = os.path.join(self.dir, "roles.yaml")
        if not os.path.exists(p):
            return {}
        return {str(k).lower(): v for k, v in (load_yaml(p).get("roles") or {}).items()}

    def profile(self, role):
        return load_yaml(os.path.join(self.dir, f"{role}.yaml"))

    def local_path(self, role):
        return os.path.join(self.local_dir, f"{role}.yaml")

    def local(self, role):
        p = self.local_path(role)
        return (load_yaml(p).get("cfg") or {}) if os.path.exists(p) else {}

    def cfg(self, role):
        """config da spingere: profilo del repo + calibrazione locale sopra."""
        c = dict(self.profile(role).get("cfg") or {})
        c.update(self.local(role))
        return c

    def save_local(self, role, values):
        os.makedirs(self.local_dir, exist_ok=True)
        cur = self.local(role)
        cur.update(values)
        with open(self.local_path(role), "w", encoding="utf-8") as f:
            f.write("# calibrazione presa al BANCO. Ricopiare in AIRA_Robot/periph/"
                    f"{role}.yaml e committare: la fonte di verita' e' il repo.\n")
            yaml.safe_dump({"cfg": cur}, f, sort_keys=True)


# ---------------------------------------------------------------------- rete
class Link:
    """Socket UDP unico sulla porta degli annunci: riceve hello/tel/log, manda hb e comandi."""

    def __init__(self, log):
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", HELLO_PORT))
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self.micros = {}          # mac -> hello + "seen"
        self.target_ip = None     # micro agganciato
        self.tel = {}
        self.tel_time = 0.0
        self.imu = {}             # ultimo {"t":"imu"} del micro agganciato
        self.hb_enabled = True
        self.on_hello = None      # callback(mac, hello) chiamata dal thread di rete
        self.on_ack = None
        # registro della telemetria da ARMATO, per tarare l'anello sui numeri veri (20/10). Un file
        # per sessione (prima si azzerava a ogni avvio); tetto TEL_LOG_MAX per non riempire il disco.
        # ~/.aira/bench_tel.jsonl resta come link all'ultima sessione
        try:
            self.tel_log = open_session_file(TEL_DIR, "tel", "jsonl", "wb")
            try:
                if os.path.islink(TEL_LATEST) or os.path.exists(TEL_LATEST):
                    os.remove(TEL_LATEST)
                os.symlink(self.tel_log.name, TEL_LATEST)
            except (OSError, NotImplementedError):
                pass
        except OSError:
            self.tel_log = None
        self.extra_log = None     # file aperto dall'autocalibrazione: telemetria completa, anche da disarmato
        self._stop = threading.Event()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._hb, daemon=True).start()

    def close(self):
        self._stop.set()

    def send(self, obj):
        ip = self.target_ip
        if ip:
            try:
                self.sock.sendto(json.dumps(obj).encode(), (ip, CMD_PORT))
            except OSError as exc:
                self.log(f"[banco] invio fallito: {exc}")

    def _hb(self):
        while not self._stop.is_set():
            if self.hb_enabled and self.target_ip:
                self.send({"t": "hb", "boss": "banco"})
            time.sleep(1.0 / HB_HZ)

    def _rx(self):
        while not self._stop.is_set():
            try:
                data, (ip, _port) = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                msg = json.loads(data.decode("utf-8", "replace"))
            except json.JSONDecodeError:
                continue
            t = msg.get("t")
            if t == "hello":
                mac = str(msg.get("mac", "?")).lower()
                msg["ip_src"] = ip
                msg["seen"] = time.time()
                with self.lock:
                    old = self.micros.get(mac, {}).get("imu")
                    self.micros[mac] = msg
                if ip == self.target_ip and msg.get("imu") != old:
                    self.log(f"[micro] IMU: {msg.get('imu')}")
                if self.on_hello:
                    self.on_hello(mac, msg)
            elif ip != self.target_ip:
                continue
            elif t == "tel":
                with self.lock:
                    self.tel = msg
                    self.tel_time = time.time()
                if self.tel_log and msg.get("st") == "armato" and self.tel_log.tell() < TEL_LOG_MAX:
                    self.tel_log.write(data.rstrip(b"\0\r\n") + b"\n")
                    self.tel_log.flush()
                extra = self.extra_log
                if extra:
                    try:
                        extra.write(data.rstrip(b"\0\r\n") + b"\n")
                    except (ValueError, OSError):   # chiuso dall'altro thread a fine prova
                        pass
            elif t == "imu":
                # 100 Hz anche da disarmato (25/10): la IMU si prova a busto fermo. Nel registro
                # della sessione insieme alla telemetria, stesso tetto
                with self.lock:
                    self.imu = msg
                if self.tel_log and self.tel_log.tell() < TEL_LOG_MAX:
                    self.tel_log.write(data.rstrip(b"\0\r\n") + b"\n")
            elif t == "log":
                self.log(f"[micro] {msg.get('m', '')}")
            elif t == "ack":
                bad = msg.get("bad") or []
                self.log(f"[banco] config applicata: {msg.get('ok')} chiavi"
                         + (f", RIFIUTATE: {', '.join(bad)}" if bad else ""))
                if self.on_ack:
                    self.on_ack(msg)
            elif t == "cfg_dump":
                items = ", ".join(f"{k}={v}" for k, v in sorted(msg.items()) if k != "t")
                self.log(f"[micro] config attuale: {items}")


def stick_dz(v):
    """zona morta dello stick, riscalata: niente salto all'uscita, a fondo resta 1."""
    a = abs(v)
    if a < STICK_DZ:
        return 0.0
    return math.copysign(min(1.0, (a - STICK_DZ) / (1.0 - STICK_DZ)), v)


def local_ip_towards(ip):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((ip, CMD_PORT))
        return s.getsockname()[0]
    finally:
        s.close()


# ---------------------------------------------------------------------- GUI
class Bench:
    def __init__(self, args, node=None):
        self.args = args
        self.node = node
        self.prof = Profiles(args.repo)
        self.role = None
        self.mac = None
        self.cfg_sent = False
        self.estop = False
        self.joy = {"left": (0.0, 0.0, 0.0), "right": (0.0, 0.0, 0.0)}
        self.joy_time = 0.0
        self.capture = {}          # ("pitch","A") -> (raw, angolo)
        self.http = None

        try:                       # copia su file della scheda log, una per sessione
            self.log_file = open_session_file(LOG_DIR, "log", "txt", "w")
        except OSError:
            self.log_file = None
        self.root = tk.Tk()
        self.root.title("AIRA — banco micro")
        self.root.configure(bg=BG)
        self._build()
        # schermo intero: il telecomando ha 800x480 e la plancia ci sta giusta (08/10)
        if not getattr(args, "windowed", False):
            self.root.attributes("-fullscreen", True)

        self.link = Link(self.log)
        self.link.on_hello = lambda mac, h: self.root.after(0, self._on_hello, mac, h)
        self.link.on_ack = lambda m: None

        if node is not None:
            # depth 1: degli stick conta solo l'ultimo valore, una coda piu' lunga = ritardo
            node.create_subscription(Point, "left_joystick_data",
                                     lambda m: self._on_joy("left", m), 1)
            node.create_subscription(Point, "right_joystick_data",
                                     lambda m: self._on_joy("right", m), 1)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(Bool, "/emergency_stop", self._on_estop, latched)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.log(f"[banco] sessione {SESSION}: log in {LOG_DIR}, telemetria in {TEL_DIR}")
        self.log(f"[banco] profili da {self.prof.dir}"
                 + ("" if node else "  (SENZA ROS: comandano gli slider)"))
        self.log("[banco] barre: verde = misurato, blu = target, giallo = setpoint (rampa)")
        self.log("[banco] in attesa che un micro si presenti sulla porta "
                 f"{HELLO_PORT}...")

    # ------------------------------------------------------------ costruzione
    def _btn(self, parent, text, cmd, **kw):
        return tk.Button(parent, text=text, command=cmd, bg=BTN, fg=FG, relief=tk.FLAT,
                         activebackground=GRID, **kw)

    def _slider(self, parent, label, lo, hi, step, length, font=("TkFixedFont", 9), width=15):
        """Slider con nome a SINISTRA e valore a DESTRA su una riga sola, sopra la barra (23/10,
        Giulio): il valore di Tk sta sopra il cursore, sotto il dito. Ritorna (riquadro, Scale)."""
        box = tk.Frame(parent, bg=BG)
        head = tk.Frame(box, bg=BG)
        head.pack(fill=tk.X)
        tk.Label(head, text=label, bg=BG, fg=FG, font=font).pack(side=tk.LEFT)
        val = tk.Label(head, text="", bg=BG, fg=SP_COL, font=font)
        val.pack(side=tk.RIGHT)
        digits = max(0, -math.floor(math.log10(step) + 1e-9)) if step < 1 else 0
        var = tk.DoubleVar(value=lo)
        # la variabile cambia sia trascinando sia con set() da programma: il numero segue sempre
        var.trace_add("write", lambda *_: val.config(text=f"{var.get():.{digits}f}"))
        sc = tk.Scale(box, from_=lo, to=hi, resolution=step, orient=tk.HORIZONTAL, length=length,
                      variable=var, showvalue=0, bg=BG, fg=FG, troughcolor=BTN,
                      highlightthickness=0, width=width)
        sc.pack(fill=tk.X)
        var.set(max(lo, min(hi, 0.0)))
        return box, sc

    def _build(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill=tk.X, padx=8, pady=(6, 2))
        self.who = tk.Label(top, text="nessun micro agganciato", bg=BG, fg=FG, anchor="w",
                            font=("TkFixedFont", 10))
        self.who.pack(side=tk.LEFT, fill=tk.X, expand=True)
        # a schermo intero non c'e' la X della finestra (e il telecomando non ha tastiera):
        # si chiude da qui, che passa da _on_close = disarmo prima di uscire
        tk.Button(top, text="chiudi", command=self._on_close, bg=ERR_COL, fg=INK, relief=tk.FLAT,
                  font=("TkDefaultFont", 9, "bold")).pack(side=tk.RIGHT)

        main = tk.Frame(self.root, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=8)

        left = tk.Frame(main, bg=BG)
        left.pack(side=tk.LEFT, anchor="n")
        self.state_lbl = tk.Label(left, text="—", bg=OFF_COL, fg=INK, width=30,
                                  font=("TkDefaultFont", 11, "bold"))
        self.state_lbl.pack(fill=tk.X, pady=(0, 6))

        row = tk.Frame(left, bg=BG)
        row.pack(fill=tk.X)
        self.arm_btn = tk.Button(row, text="ARMA", bg=WARN_COL, fg=INK, relief=tk.FLAT,
                                 font=("TkDefaultFont", 12, "bold"), width=10,
                                 command=self._toggle_arm)
        self.arm_btn.pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value="pos")
        for txt, val in (("posizione", "pos"), ("RAW motori", "raw")):
            tk.Radiobutton(row, text=txt, value=val, variable=self.mode_var, bg=BG, fg=FG,
                           selectcolor=BTN, activebackground=BG,
                           command=self._set_mode).pack(side=tk.LEFT, padx=4)

        row2 = tk.Frame(left, bg=BG)
        row2.pack(fill=tk.X, pady=4)
        self._btn(row2, "clear", lambda: self.link.send({"t": "clear"})).pack(side=tk.LEFT)
        self.hb_btn = self._btn(row2, "stacca HB", self._toggle_hb)
        self.hb_btn.pack(side=tk.LEFT, padx=4)
        self._btn(row2, "rimanda cfg", self._push_cfg).pack(side=tk.LEFT)
        self._btn(row2, "leggi cfg", lambda: self.link.send({"t": "get"})).pack(side=tk.LEFT, padx=4)

        # barre giunti
        self.bars = {}
        # ⚠️ lo schermo del telecomando e' 800x480 (finestra utile ~798x416): la colonna deve
        # restare sotto ~390 px. Nomi accanto alle barre, motori affiancati, slider in una scheda.
        for j, span in (("pitch", 25.0), ("roll", 20.0)):
            r = tk.Frame(left, bg=BG)
            r.pack(fill=tk.X, pady=(5, 0))
            tk.Label(r, text=j.upper(), bg=BG, fg=FG, width=6, anchor="w",
                     font=("TkDefaultFont", 9, "bold")).pack(side=tk.LEFT)
            c = tk.Canvas(r, width=280, height=24, bg=BG, highlightthickness=1,
                          highlightbackground=GRID)
            c.pack(side=tk.LEFT)
            lbl = tk.Label(left, text="", bg=BG, fg=FG, font=("TkFixedFont", 8), anchor="w", justify="left")
            lbl.pack(fill=tk.X)
            self.bars[j] = (c, lbl, span)

        r = tk.Frame(left, bg=BG)
        r.pack(fill=tk.X, pady=(6, 0))
        self.duty_bars = {}
        for m in ("A", "B"):
            tk.Label(r, text=f"mot {m}", bg=BG, fg=FG, font=("TkDefaultFont", 9, "bold")).pack(
                side=tk.LEFT, padx=(0 if m == "A" else 6, 2))
            c = tk.Canvas(r, width=118, height=18, bg=BG, highlightthickness=1,
                          highlightbackground=GRID)
            c.pack(side=tk.LEFT)
            self.duty_bars[m] = c
        self.motor_lbl = tk.Label(left, text="", bg=BG, fg=FG, font=("TkFixedFont", 8),
                                  anchor="w", justify="left")
        self.motor_lbl.pack(fill=tk.X)

        # destra: calibrazione + OTA + log
        st = ttk.Style(self.root)
        st.theme_use("default")
        st.configure("TNotebook", background=BG, borderwidth=0)
        st.configure("TNotebook.Tab", background=BTN, foreground=FG, padding=(10, 4))
        st.map("TNotebook.Tab", background=[("selected", GRID)])
        self.tabs = ttk.Notebook(main)
        self.tabs.pack(side=tk.LEFT, anchor="n", padx=(6, 0), fill=tk.BOTH, expand=True)
        tab_log = tk.Frame(self.tabs, bg=BG)
        self.tab_micro = tk.Frame(self.tabs, bg=BG)
        tab_cal = tk.Frame(self.tabs, bg=BG)
        self.tabs.add(tab_log, text="log")
        self.tabs.add(self.tab_micro, text="micro")
        self.tabs.add(tab_cal, text="cal / OTA")
        tab_sl = tk.Frame(self.tabs, bg=BG)
        self.tabs.add(tab_sl, text="slider")

        # slider (senza ROS o senza joystick)
        tk.Label(tab_sl, text="comandano se il joystick tace da 1 s, o sempre con la casella",
                 bg=BG, fg=GRID, font=("TkFixedFont", 8)).pack(anchor="w", pady=(4, 0))
        # con la casella gli stick sono esclusi: duty/angolo FISSO a mani libere (tester, ACS712,
        # prove di durata). Gli stick sul telecomando pubblicano sempre, anche fermi (19/10)
        self.use_sl = tk.BooleanVar(value=False)
        tk.Checkbutton(tab_sl, text="comanda con gli slider (stick esclusi)", variable=self.use_sl,
                       command=self._toggle_sliders, bg=BG, fg=FG, selectcolor=BTN,
                       activebackground=BG, activeforeground=FG).pack(anchor="w")
        # RAW a fondo = 100% di duty: alza INSIEME raw_span (qui) e raw_max (nel micro, via cfg),
        # che di norma tagliano entrambi al 30%. Si accende solo da disarmato (19/10)
        self.full_duty = tk.BooleanVar(value=False)
        tk.Checkbutton(tab_sl, text="RAW: a fondo = 100% duty (di norma 30%)", variable=self.full_duty,
                       command=self._toggle_full_duty, bg=BG, fg=FG, selectcolor=BTN,
                       activebackground=BG, activeforeground=FG).pack(anchor="w")
        # diagnosi: in RAW il micro ignora pot e limiti (cfg raw_ignore_pot), resta solo la
        # sovracorrente. Serve dopo un cambio motori, quando il busto va sui limiti logici in
        # un secondo e ogni colpo finisce in fault (21/10). Si accende solo da disarmato
        self.no_lim = tk.BooleanVar(value=False)
        tk.Checkbutton(tab_sl, text="RAW: ignora pot e limiti (diagnosi)", variable=self.no_lim,
                       command=self._toggle_no_lim, bg=BG, fg=FG, selectcolor=BTN,
                       activebackground=BG, activeforeground=FG).pack(anchor="w")
        self.sl = {}
        for name in ("pitch / mot A", "roll / mot B"):
            box, sc = self._slider(tab_sl, name, -1.0, 1.0, 0.01, 360)
            box.pack(pady=2)
            self.sl[name] = sc
        self._btn(tab_sl, "slider a zero",
                  lambda: [sc.set(0) for sc in self.sl.values()]).pack(pady=4)

        self._build_funcs()

        # scheda PID: si manda al RILASCIO dello slider (set() da programma non manda nulla)
        tab_pid = tk.Frame(self.tabs, bg=BG)
        self.tabs.add(tab_pid, text="PID")
        grid = tk.Frame(tab_pid, bg=BG)
        grid.pack(fill=tk.X)
        self.tune_sl = {}
        for n, item in enumerate(TUNE):
            if item is None:
                continue
            key, label, lo, hi, step = item
            box, sc = self._slider(grid, label, lo, hi, step, 132, font=("TkFixedFont", 8), width=10)
            box.grid(row=n // 3, column=n % 3, padx=2, pady=1, sticky="we")   # 3 colonne: una riga per gruppo
            sc.bind("<ButtonRelease-1>", functools.partial(self._tune_send, key))
            self.tune_sl[key] = sc
        row = tk.Frame(tab_pid, bg=BG)
        row.pack(fill=tk.X, pady=2)
        self._btn(row, "valori del profilo", self._tune_reset).pack(side=tk.LEFT)
        self._btn(row, "scrivi valori nel log", self._tune_dump).pack(side=tk.LEFT, padx=4)

        # una riga per ogni scheda che si annuncia (si ricostruisce solo quando cambia qualcosa)
        self.micro_frame = tk.Frame(self.tab_micro, bg=BG)
        self.micro_frame.pack(fill=tk.BOTH, expand=True, pady=4)
        self.micro_sig = None

        cal = tk.LabelFrame(tab_cal, text="calibrazione pot (due pose, angolo dalla livella)",
                            bg=BG, fg=FG)
        cal.pack(fill=tk.X)
        self.cal_entries = {}
        for j in ("pitch", "roll"):
            r = tk.Frame(cal, bg=BG)
            r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=f"{j:5}", bg=BG, fg=FG, font=("TkFixedFont", 9)).pack(side=tk.LEFT)
            for pt in ("A", "B"):
                e = tk.Entry(r, width=6, bg=BTN, fg=FG, insertbackground=FG)
                e.insert(0, "-10" if pt == "A" else "10")
                e.pack(side=tk.LEFT, padx=(6, 0))
                self.cal_entries[(j, pt)] = e
                self._btn(r, f"cattura {pt}",
                          functools.partial(self._capture, j, pt)).pack(side=tk.LEFT, padx=2)
        self.cal_lbl = tk.Label(cal, text="", bg=BG, fg=FG, font=("TkFixedFont", 9), anchor="w",
                                justify="left")
        self.cal_lbl.pack(fill=tk.X)
        self._btn(cal, "salva calibrazione e spingi", self._save_cal).pack(anchor="w", pady=2)

        ota = tk.Frame(tab_cal, bg=BG)
        ota.pack(fill=tk.X, pady=6)
        self._btn(ota, "OTA: scegli .bin", self._ota).pack(side=tk.LEFT)
        self._btn(ota, "riavvia micro", self._reboot).pack(side=tk.LEFT, padx=4)

        self.logbox = scrolledtext.ScrolledText(tab_log, width=47, height=20, bg=INK, fg=FG,
                                                font=("TkFixedFont", 9))
        self.logbox.pack(fill=tk.BOTH, expand=True)

    def _build_funcs(self):
        tab = tk.Frame(self.tabs, bg=BG)
        self.tabs.add(tab, text="funzioni")
        self.func_sel = tk.StringVar(value="manual")
        sel = tk.Frame(tab, bg=BG)
        sel.pack(fill=tk.X, pady=(4, 0))
        for n, (val, txt) in enumerate(FUNCS):
            tk.Radiobutton(sel, text=txt, value=val, variable=self.func_sel, bg=BG, fg=FG,
                           selectcolor=BTN, activebackground=BG, activeforeground=FG,
                           command=self._func_selected).grid(row=n // 2, column=n % 2, sticky="w")
        row = tk.Frame(tab, bg=BG)
        row.pack(fill=tk.X, pady=4)
        self.func_btn = tk.Button(row, text="AVVIA", bg=OK_COL, fg=INK, relief=tk.FLAT, width=8,
                                  font=("TkDefaultFont", 11, "bold"), command=self._func_toggle)
        self.func_btn.pack(side=tk.LEFT)
        # caselle che dipendono dalla funzione scelta (una sola visibile per volta)
        self.func_ccw = tk.BooleanVar(value=False)
        self.ac_raw = tk.BooleanVar(value=True)
        self.func_checks = {
            "twirl": tk.Checkbutton(row, text="twirl antiorario", variable=self.func_ccw, bg=BG, fg=FG,
                                    selectcolor=BTN, activebackground=BG, activeforeground=FG),
            "autocal": tk.Checkbutton(row, text="includi RAW (stacco)", variable=self.ac_raw, bg=BG,
                                      fg=FG, selectcolor=BTN, activebackground=BG, activeforeground=FG)}
        self.func_lbl = tk.Label(tab, text="", bg=BG, fg=SP_COL, font=("TkFixedFont", 9), anchor="w",
                                 justify="left")
        self.func_lbl.pack(fill=tk.X)
        # slider: valgono subito, anche in corsa (la funzione li rilegge a ogni giro). Si vedono
        # solo quelli della funzione scelta (FUNC_SLIDERS)
        self.func_box = tk.Frame(tab, bg=BG)
        self.func_box.pack(fill=tk.X)
        self.func_sl, self.func_sl_box = {}, {}
        for key, label, lo, hi, step, v0 in (
                ("period", "periodo s (un giro / un'oscillazione)", 1.0, 20.0, 0.5, 6.0),
                ("sw_p", "swing pitch deg", 0.0, 30.0, 0.5, 10.0),
                ("sw_r", "swing roll deg", 0.0, 19.0, 0.5, 8.0),
                ("pos_p", "pitch fisso deg", -25.0, 30.0, 0.5, 0.0),
                ("pos_r", "roll fisso deg", -19.0, 19.0, 0.5, 0.0)):
            box, sc = self._slider(self.func_box, label, lo, hi, step, 360)
            sc.set(v0)
            self.func_sl[key], self.func_sl_box[key] = sc, box
        self.func_note = tk.Label(tab, text="", bg=BG, fg=GRID, font=("TkFixedFont", 8),
                                  anchor="w", justify="left")
        self.func_note.pack(fill=tk.X, pady=(4, 0))
        self.func = None           # funzione in corso (dict con fase e stato); None = manuale
        self.autocal = None        # autocalibrazione in corso (autocal.Autocal)
        self._func_layout()

    def _func_layout(self):
        kind = self.func_sel.get()
        for key, box in self.func_sl_box.items():
            box.pack_forget()
        keys = FUNC_SLIDERS.get(kind, ())
        # un riquadro rimasto vuoto in Tk non si restringe: lo si toglie del tutto
        if keys:
            self.func_box.pack(fill=tk.X, before=self.func_note)
        else:
            self.func_box.pack_forget()
        for key in keys:
            self.func_sl_box[key].pack(pady=2)
        for k, cb in self.func_checks.items():
            if k == kind:
                cb.pack(side=tk.LEFT, padx=8)
            else:
                cb.pack_forget()
        self.func_note.config(text=FUNC_NOTES.get(kind, ""))

    # ------------------------------------------------------------ funzioni automatiche
    def _running_kind(self):
        return "autocal" if self.autocal else (self.func["kind"] if self.func else None)

    def _func_selected(self):
        running = self._running_kind()
        kind = self.func_sel.get()
        if running and kind != running:
            if kind == "manual" and self.func:
                self._func_stop("scelto Manuale")
            else:
                self.func_sel.set(running)      # un'altra funzione: prima si ferma questa
                self.log("[funzioni] prima ferma quella in corso (STOP)")
        self._func_layout()

    def _func_toggle(self):
        if self.autocal:
            self._autocal_end("fermata con STOP")
            return
        if self.func:
            self._func_stop("stop")
            return
        kind = self.func_sel.get()
        if kind == "manual":
            self.log("[funzioni] scegli una funzione (Manuale = joystick, gia' attivo)")
            return
        tel, age = self._tel()
        if age > 0.5 or tel.get("st") != "armato" or tel.get("mode") != "pos":
            messagebox.showwarning("Funzioni", "Serve il micro ARMATO in modo posizione.")
            return
        if kind == "autocal":
            self._autocal_start(tel)
            return
        # si parte dalla posa MISURATA: il target non salta
        self.func = {"kind": kind, "phase": "avvicinamento", "p": float(tel.get("p", 0.0)),
                     "r": float(tel.get("r", 0.0)), "th": 0.0, "w": 0.0, "amp": None,
                     "t": time.time()}
        if kind == "twirl":
            # si entra nell'ellisse dal punto piu' vicino alla posa attuale (meno strada)
            ap, ar, _cp, _cr = self._func_shape()
            p, r = self.func["p"], self.func["r"]
            if ap > 0 and ar > 0 and (p or r):
                self.func["th"] = math.atan2(r / ar, p / ap)
        self.func_btn.config(text="STOP", bg=ERR_COL)
        self._func_mark("start")
        self.log(f"[funzioni] {kind}: avvicinamento al punto di partenza")

    def _func_stop(self, why):
        if self.func and self.func["phase"] != "ritorno":
            self.func["phase"] = "ritorno"
            self._func_mark("stop")
            self.log(f"[funzioni] {why}: ritorno piano al centro")

    def _func_end(self, why=None):
        """Fine della funzione. Con `why` (disarmo / fault / fungo) e' un'interruzione: niente
        ritorno al centro, il micro e' gia' fermo."""
        if self.func and why:
            self.log(f"[funzioni] interrotta ({why})")
            self._func_mark("abort")
        self.func = None
        self.func_btn.config(text="AVVIA", bg=OK_COL)
        self.func_sel.set("manual")
        self._func_layout()

    def _func_mark(self, ev):
        f = self.func or {}
        self._tel_mark({"t": "func", "ev": ev, "kind": f.get("kind"),
                        **{k: float(sc.get()) for k, sc in self.func_sl.items()},
                        "ccw": bool(self.func_ccw.get())})

    def _func_shape(self):
        """(ampiezza pitch, ampiezza roll, centro pitch, centro roll) della funzione.
        Oscillatore pitch: oscilla il pitch, il roll sta FERMO al valore scelto (e viceversa)."""
        kind = self.func["kind"] if self.func else self.func_sel.get()
        g = {k: float(sc.get()) for k, sc in self.func_sl.items()}
        if kind == "twirl":
            return g["sw_p"], g["sw_r"], 0.0, 0.0
        if kind == "osc_p":
            return g["sw_p"], 0.0, 0.0, g["pos_r"]
        return 0.0, g["sw_r"], g["pos_p"], 0.0

    @staticmethod
    def _approach(cur, goal, step):
        dp, dr = goal[0] - cur[0], goal[1] - cur[1]
        d = math.hypot(dp, dr)
        if d <= step:
            return goal, True
        return (cur[0] + dp / d * step, cur[1] + dr / d * step), False

    def _func_point(self, f):
        """Posa della funzione all'angolo th. Twirl: ellisse pitch = cos, roll = sin (il verso lo
        da' il segno della velocita'). Oscillatori: coseno sul loro asse, cosi' si parte dal picco
        a velocita' zero."""
        ap, ar, cp, cr = f["amp"]
        th = f["th"]
        if f["kind"] == "twirl":
            return cp + ap * math.cos(th), cr + ar * math.sin(th)
        return cp + ap * math.cos(th), cr + ar * math.cos(th)

    def _func_step(self):
        """Un giro (20 Hz): aggiorna la funzione e ritorna il target (pitch, roll)."""
        f = self.func
        now = time.time()
        dt = min(0.2, now - f["t"])
        f["t"] = now
        want = self._func_shape()
        if f["amp"] is None:
            f["amp"] = want
        # swing / posizione fissa cambiati dallo slider: ci si arriva a velocita' limitata
        step = FUNC_AMP_RATE * dt
        f["amp"] = tuple(a + max(-step, min(step, w - a)) for a, w in zip(f["amp"], want))
        if f["phase"] == "avvicinamento":
            (f["p"], f["r"]), arrived = self._approach((f["p"], f["r"]), self._func_point(f),
                                                       FUNC_APPROACH * dt)
            if arrived:
                f["phase"] = "in corsa"
                self.log(f"[funzioni] {f['kind']}: partita")
        elif f["phase"] == "in corsa":
            w_full = 2.0 * math.pi / max(0.5, float(self.func_sl["period"].get()))
            if f["kind"] == "twirl" and self.func_ccw.get():
                w_full = -w_full
            # avvio (e cambi di periodo) dolci: la velocita' angolare sale in FUNC_SPINUP_S
            dw = abs(w_full) / FUNC_SPINUP_S * dt
            f["w"] += max(-dw, min(dw, w_full - f["w"]))
            f["th"] += f["w"] * dt
            f["p"], f["r"] = self._func_point(f)
        else:                                   # ritorno al centro
            (f["p"], f["r"]), arrived = self._approach((f["p"], f["r"]), (0.0, 0.0), FUNC_APPROACH * dt)
            if arrived:
                self.log("[funzioni] al centro: comanda di nuovo il joystick")
                self._func_end()
                return 0.0, 0.0
        return f["p"], f["r"]

    # ------------------------------------------------------------ autocalibrazione
    def _autocal_start(self, tel):
        if not tel.get("cal"):
            messagebox.showwarning("Autocalibrazione", "Pot non calibrati.")
            return
        raw = bool(self.ac_raw.get())
        if not messagebox.askyesno(
                "Autocalibrazione",
                f"Il busto si muove da solo per circa {5 if raw else 4} minuti"
                + (", anche in RAW (motori diretti, stop a 2 deg pitch / 5 deg roll)" if raw else "")
                + ".\nBusto libero, niente mani in mezzo, fungo a portata. Procedere?"):
            return
        tune = {k: float(sc.get()) for k, sc in self.tune_sl.items()}
        try:
            self.autocal = autocal.Autocal(getattr(self, "mix", (1.0, 1.0, 1.0)), raw, tune,
                                           self._fw_of_attached(), self.log)
        except OSError as exc:
            messagebox.showwarning("Autocalibrazione", f"Non riesco a creare il file: {exc}")
            return
        try:
            self.link.extra_log = open(self.autocal.tel_path, "wb")
        except OSError:
            self.link.extra_log = None
        self._tel_mark({"t": "func", "ev": "start", "kind": "autocal", "file": self.autocal.path})
        self.func_btn.config(text="STOP", bg=ERR_COL)
        self.log(f"[autocal] partita, risultati in {self.autocal.path}")

    def _fw_of_attached(self):
        with self.link.lock:
            return (self.link.micros.get(self.mac) or {}).get("fw")

    def _autocal_tick(self, tel, live):
        """Un giro (20 Hz) dell'autocalibrazione: la sequenza decide cosa mandare."""
        if self.estop:
            self._autocal_end("fungo")
            return
        if not live:
            self.ac_offline = getattr(self, "ac_offline", 0) + 1
            if self.ac_offline > 20:           # 1 s senza telemetria
                self._autocal_end("telemetria persa")
            return
        self.ac_offline = 0
        try:
            msg = self.autocal.step(tel)
        except autocal.Abort as exc:
            self._autocal_end(str(exc))
            return
        except StopIteration:
            self._autocal_end(None)
            return
        if msg:
            self.link.send(msg)

    def _autocal_end(self, why):
        ac, self.autocal = self.autocal, None
        if ac is None:
            return
        if why:
            # interrotta: motori a zero, disarmo e di nuovo in modo posizione (da disarmati si puo')
            self.link.send({"t": "raw", "a": 0.0, "b": 0.0})
            self.link.send({"t": "arm", "on": False})
            self.link.send({"t": "mode", "m": "pos"})
            self.log(f"[autocal] INTERROTTA ({why}): disarmato")
        ac.finish(why)
        extra, self.link.extra_log = self.link.extra_log, None
        if extra:
            extra.close()
        self._tel_mark({"t": "func", "ev": "abort" if why else "end", "kind": "autocal"})
        self.func_btn.config(text="AVVIA", bg=OK_COL)
        self.func_sel.set("manual")
        self._func_layout()

    def _func_show(self):
        if self.autocal:
            self.func_lbl.config(text=self.autocal.status())
            return
        f = self.func
        if not f:
            self.func_lbl.config(text="comanda il joystick" if self.func_sel.get() == "manual"
                                 else "pronta: AVVIA (micro armato, modo posizione)")
            return
        self.func_lbl.config(text=f"{f['kind']}: {f['phase']}   target p {f['p']:+6.1f}  r {f['r']:+6.1f}")

    # ------------------------------------------------------------ log (thread-safe)
    def log(self, text):
        line = time.strftime("%H:%M:%S ") + text + "\n"
        try:
            self.root.after(0, self._append_log, line)
        except RuntimeError:
            pass

    def _append_log(self, line):
        if self.log_file:
            try:
                self.log_file.write(line)
                self.log_file.flush()
            except (OSError, ValueError):
                pass
        self.logbox.insert(tk.END, line)
        if int(self.logbox.index("end-1c").split(".")[0]) > 2000:
            self.logbox.delete("1.0", "500.0")
        self.logbox.see(tk.END)

    # ------------------------------------------------------------ aggancio
    def _on_hello(self, mac, h):
        if mac == self.mac:
            self.link.target_ip = h["ip_src"]
            # il micro si e' riavviato o ci ha perso: la config va rispinta
            if h.get("boss") == "nessuno" and self.link.hb_enabled:
                self.cfg_sent = False
            self.last_hello = h
            self._who(h)
            return
        if self.mac is None:
            self._auto_attach()

    def _micros(self):
        """schede viste: [(mac, hello, ruolo o None)], prima le note, poi per ruolo e MAC."""
        roles = self.prof.roles()
        now = time.time()
        with self.link.lock:
            ms = {m: dict(h) for m, h in self.link.micros.items()}
        out = [(m, h, roles.get(m)) for m, h in ms.items()]
        out.sort(key=lambda x: (x[2] is None, x[2] or "", x[0]))
        for _m, h, _r in out:
            h["gone"] = now - h.get("seen", 0) > MICRO_GONE_S
        return out

    def _auto_attach(self):
        """UN solo micro in rete e con ruolo noto -> ci si aggancia da soli (come al banco con
        una scheda). Con piu' schede, o una sconosciuta, si aspetta la scelta in lista."""
        live = [(m, h, r) for m, h, r in self._micros() if not h["gone"]]
        if len(live) == 1 and live[0][2]:
            self._attach(*live[0])
        elif live and not getattr(self, "_told_choose", False):
            self._told_choose = True
            self.tabs.select(self.tab_micro)
            self.log(f"[banco] {len(live)} schede in rete (o una sconosciuta): scegli dalla "
                     "lista quale agganciare")

    def _who(self, h):
        role = self.role or "SCONOSCIUTO"
        self.who.config(text=f"AGGANCIATO: {role}  MAC {h.get('mac')}  IP {h.get('ip_src')}  "
                             f"fw {h.get('fw')} [{h.get('part')}, {h.get('img')}]  "
                             f"capo: {h.get('boss')}")

    def _choose(self, mac, role):
        """pulsante "aggancia" di una riga della lista."""
        if mac == self.mac:
            return
        if self.mac is not None:
            tel, age = self._tel()
            if age < 0.5 and tel.get("st") == "armato" and not messagebox.askyesno(
                    "Cambio micro", f"'{self.role}' e' ARMATO.\nDisarmarlo e passare all'altro?"):
                return
            self._detach()
        with self.link.lock:
            h = dict(self.link.micros.get(mac, {}))
        if h:
            if not role:
                self.log(f"[banco] {mac} non e' in periph/roles.yaml: lo tratto come BUSTO solo "
                         f"per questa sessione (aggiungi '\"{mac}\": torso' al repo)")
            self._attach(mac, h, role or "torso")

    def _detach(self):
        """lascia il micro agganciato: disarmo, poi niente piu' heartbeat -> resta senza capo."""
        if self.mac is None:
            return
        self.link.send({"t": "arm", "on": False})
        self.log(f"[banco] lascio {self.mac} ('{self.role}'): disarmato, resta senza capo")
        self.link.target_ip = None
        with self.link.lock:
            self.link.tel = {}
            self.link.tel_time = 0.0
        self.mac = self.role = None
        self.cfg_sent = False
        self.capture = {}
        self.cal_lbl.config(text="")
        self.who.config(text="nessun micro agganciato")
        self.micro_sig = None

    def _attach(self, mac, h, role):
        try:
            prof = self.prof.profile(role)
        except (OSError, RuntimeError) as exc:
            messagebox.showerror("Profilo", f"Non leggo il profilo '{role}':\n{exc}")
            return
        if h.get("boss") not in (None, "nessuno", "banco"):
            self.log(f"[banco] ⚠ {mac} ha gia' un capo ({h.get('boss')}): il mini PC e' sullo "
                     "switch insieme al banco? Mai tutti e due (handbook §14.2)")
        self.mac = mac
        self.role = role
        self.bench_cfg = prof.get("bench") or {}
        # solo ORA si diventa capo: a una scheda senza ruolo non si manda nemmeno l'heartbeat
        self.link.target_ip = h["ip_src"]
        self.log(f"[banco] agganciato {mac} come '{role}'")
        self.cfg_sent = False
        self.last_hello = h
        self._who(h)
        self.micro_sig = None

    def _refresh_micros(self):
        ms = self._micros()
        sig = tuple((m, r, h.get("ip_src"), h.get("fw"), h.get("part"), h.get("img"),
                     h.get("boss"), h.get("imu"), h["gone"]) for m, h, r in ms) + (self.mac,)
        if sig == self.micro_sig:
            return
        self.micro_sig = sig
        for w in self.micro_frame.winfo_children():
            w.destroy()
        if not ms:
            tk.Label(self.micro_frame, text="nessuna scheda si e' ancora annunciata", bg=BG,
                     fg=GRID, font=("TkFixedFont", 9)).pack(anchor="w")
            return
        for m, h, r in ms:
            mine = m == self.mac
            boss = h.get("boss")
            foreign = boss not in (None, "nessuno", "banco")
            fg = OFF_COL if h["gone"] else (ERR_COL if foreign else FG)
            bg = GRID if mine else BG
            row = tk.Frame(self.micro_frame, bg=bg)
            row.pack(fill=tk.X)
            line1 = f"{(r or 'SCONOSCIUTO'):12} {m}" + ("  (assente)" if h["gone"] else "")
            line2 = (f"  {h.get('ip_src', '?'):15} fw {h.get('fw', '?')} {h.get('part', '?')}"
                     f"  capo: {boss}" + (f"  imu: {h['imu']}" if h.get("imu") else ""))
            tk.Label(row, text=line1 + "\n" + line2, bg=bg, fg=fg, font=("TkFixedFont", 9),
                     anchor="w", justify="left").pack(side=tk.LEFT, fill=tk.X, expand=True)
            if mine:
                tk.Label(row, text="AGGANCIATO", bg=OK_COL, fg=INK,
                         font=("TkDefaultFont", 9, "bold")).pack(side=tk.RIGHT, padx=2)
            elif not h["gone"]:
                label = "aggancia" if r else "aggancia\ncome BUSTO"
                self._btn(row, label, functools.partial(self._choose, m, r)).pack(
                    side=tk.RIGHT, padx=2)
        live = sum(1 for _m, h, _r in ms if not h["gone"])
        self.tabs.tab(self.tab_micro, text=f"micro ({live})")

    def _push_cfg(self):
        if not self.role:
            return
        cfg = self.prof.cfg(self.role)
        # segni del mixing, per il RAW push-pull dallo stick (gli stessi che usa il firmware)
        self.mix = tuple(float(cfg.get(k, 1.0)) for k in ("mix.sa", "mix.sb", "mix.sr"))
        self._tune_show(cfg)
        if self.full_duty.get():               # la spunta sopravvive a un riaggancio del micro
            cfg["raw_max"] = 1.0
        if self.no_lim.get():
            cfg["raw_ignore_pot"] = 1
        # il micro non accetta piu' di ~1 KB per datagramma: si spezza
        items = list(cfg.items())
        for i in range(0, len(items), 12):
            msg = {"t": "cfg"}
            msg.update(dict(items[i:i + 12]))
            self.link.send(msg)
        self.cfg_sent = True
        self.log(f"[banco] config spinta ({len(items)} chiavi)")

    # ------------------------------------------------------------ comandi
    def _tel(self):
        with self.link.lock:
            return dict(self.link.tel), time.time() - self.link.tel_time

    def _toggle_arm(self):
        tel, _ = self._tel()
        if tel.get("st") == "armato":
            self.link.send({"t": "arm", "on": False})
            return
        if self.estop:
            messagebox.showwarning("Fungo", "Il fungo EM STOP e' premuto.")
            return
        # niente conferma (22/10, Giulio): al banco si arma e disarma di continuo
        if self.use_sl.get():                  # niente strappo all'arm: si parte da zero
            for sc in self.sl.values():
                sc.set(0)
        self.link.send({"t": "arm", "on": True})

    def _toggle_sliders(self):
        for sc in self.sl.values():            # al cambio di sorgente si riparte da zero
            sc.set(0)
        self.log("[banco] comandano gli SLIDER (stick esclusi)" if self.use_sl.get()
                 else "[banco] comandano gli STICK")

    def _toggle_full_duty(self):
        prof_max = float(self.prof.cfg(self.role).get("raw_max", 0.3)) if self.role else 0.3
        if not self.full_duty.get():           # ridurre e' sempre sicuro, anche da armato
            self.link.send({"t": "cfg", "raw_max": prof_max})
            self.log(f"[banco] RAW di nuovo al {prof_max:.0%} di duty")
            return
        tel, age = self._tel()
        if age < 0.5 and tel.get("st") == "armato":
            self.full_duty.set(False)
            messagebox.showwarning("100% duty", "Prima disarmare.")
            return
        if not messagebox.askyesno("100% duty", "RAW a fondo = 100% del PWM (12 V pieni).\n"
                                   "In RAW non ci sono fine-corsa: sul busto vero il motore va\n"
                                   "dritto contro le battute. Procedere?"):
            self.full_duty.set(False)
            return
        self.link.send({"t": "cfg", "raw_max": 1.0})
        self.log("[banco] RAW al 100% di duty (raw_max 1.0 nel micro)")

    # ------------------------------------------------------------ scheda PID
    def _tune_show(self, cfg):
        for key, sc in self.tune_sl.items():
            v = cfg.get("vel_max") if key == "vel" else cfg.get(key)
            if v is not None:
                sc.set(float(v))

    def _tune_send(self, key, _ev=None):
        if not self.role:
            return
        v = float(self.tune_sl[key].get())
        msg = {"t": "cfg"}
        if key == "vel":
            msg.update({"vel_max": v, "vel_banco": v})   # al banco vale il minore dei due
        else:
            msg[key] = v
        self.link.send(msg)
        self.log(f"[PID] {key} = {v:g}")
        # nel registro della telemetria, per sapere da quando valgono i valori nuovi
        self._tel_mark({"t": "tune", "key": key, "v": v})

    def _tel_mark(self, obj):
        """Riga di contesto nel registro della telemetria (cambi PID, avvio/stop funzioni)."""
        if self.link.tel_log:
            self.link.tel_log.write((json.dumps({**obj, "time": time.time()}) + "\n").encode())
            self.link.tel_log.flush()

    def _tune_reset(self):
        if not self.role:
            return
        cfg = self.prof.cfg(self.role)
        self._tune_show(cfg)
        msg = {"t": "cfg"}
        for key in self.tune_sl:
            if key == "vel":
                msg.update({"vel_max": cfg.get("vel_max"), "vel_banco": cfg.get("vel_banco")})
            elif key in cfg:
                msg[key] = cfg[key]
        self.link.send({k: v for k, v in msg.items() if v is not None})
        self.log("[PID] tornati i valori del profilo")

    def _tune_dump(self):
        vals = {key: float(sc.get()) for key, sc in self.tune_sl.items()}
        self.log("[PID] da copiare in periph/torso.yaml: " +
                 ", ".join(f"{('vel_max' if k == 'vel' else k)}: {v:g}" for k, v in vals.items()))

    def _toggle_no_lim(self):
        prof = self.prof.cfg(self.role).get("raw_ignore_pot", 0) if self.role else 0
        if not self.no_lim.get():              # rimettere i limiti e' sempre sicuro
            self.link.send({"t": "cfg", "raw_ignore_pot": prof})
            self.log("[banco] RAW: pot e limiti di nuovo attivi")
            return
        tel, age = self._tel()
        if age < 0.5 and tel.get("st") == "armato":
            self.no_lim.set(False)
            messagebox.showwarning("ignora limiti", "Prima disarmare.")
            return
        if not messagebox.askyesno("ignora limiti", "In RAW il micro non guarda piu' i pot:\n"
                                   "niente fault limite, il busto puo' arrivare alle battute.\n"
                                   "Resta solo la sovracorrente. Colpi brevi. Procedere?"):
            self.no_lim.set(False)
            return
        self.link.send({"t": "cfg", "raw_ignore_pot": 1})
        self.log("[banco] RAW: pot e limiti IGNORATI (diagnosi)")

    def _set_mode(self):
        # il micro cambia modo SOLO da disarmato e se rifiuta lo dice solo sulla sua seriale:
        # qui lo si dice a voce, e _tick riporta comunque il pallino al modo vero (19/10)
        tel, age = self._tel()
        if age < 0.5 and tel.get("st") == "armato" and tel.get("mode") != self.mode_var.get():
            self.log("[banco] cambio modo IGNORATO: prima disarmare")
        self.mode_click = time.time()
        self.link.send({"t": "mode", "m": self.mode_var.get()})

    def _toggle_hb(self):
        self.link.hb_enabled = not self.link.hb_enabled
        if self.link.hb_enabled:
            self.hb_btn.config(text="stacca HB", bg=BTN, fg=FG)
            self.log("[banco] heartbeat RIATTACCATO")
        else:
            self.hb_btn.config(text="RIATTACCA HB", bg=ERR_COL, fg=INK)
            self.log("[banco] heartbeat STACCATO: il micro deve disarmarsi da solo")

    def _reboot(self):
        if messagebox.askyesno("Riavvio", "Riavviare il micro?"):
            self.link.send({"t": "reboot"})

    def _on_joy(self, side, m):
        self.joy[side] = (m.x, m.y, m.z)
        self.joy_time = time.time()

    def _on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop:
            self.link.send({"t": "arm", "on": False})
            self.log("[banco] FUNGO premuto: disarmo")

    def _send_command(self, tel):
        if self.autocal:                       # comanda la sequenza (anche fra disarmo e riarmo)
            self._autocal_tick(tel, bool(tel))
            return
        if tel.get("st") != "armato" or self.estop:
            if self.func:
                self._func_end("fungo" if self.estop else (tel.get("f") or tel.get("st") or "offline"))
            return
        if self.func:
            if tel.get("mode") != "pos":
                self._func_end("non piu' in modo posizione")
                return
            p, r = self._func_step()          # joystick e slider ignorati
            self.link.send({"t": "cmd", "p": round(p, 2), "r": round(r, 2)})
            return
        joy_live = (self.node is not None and not self.use_sl.get()
                    and time.time() - self.joy_time < JOY_TIMEOUT)
        if joy_live:
            # zona morta: a riposo lo stick non torna a 0 (-0.03..0.04) e a span 25 deg
            # erano ~1 deg di target che ballava da solo (telemetria 20/10)
            lx, ly, rx, ry = (stick_dz(v) for v in (*self.joy["left"][:2], *self.joy["right"][:2]))
            py = ry               # in posizione tutto sullo stick DX: Y -> pitch, X -> roll (20/10)
        else:
            ly = self.sl["pitch / mot A"].get()
            rx = ry = self.sl["roll / mot B"].get()
            py = ly
        b = getattr(self, "bench_cfg", {})
        if tel.get("mode") == "raw":
            span = 1.0 if self.full_duty.get() else float(b.get("raw_span", 0.3))
            if joy_live:
                # stick DX push-pull (20/10): Y = motori insieme (pitch), X = in opposizione
                # (roll), coi segni del mixing del firmware -> destra sullo stick = destra
                sa, sb, sr = getattr(self, "mix", (1.0, 1.0, 1.0))
                ua, ub = sa * (ry + sr * rx), sb * (ry - sr * rx)
                k = max(1.0, abs(ua), abs(ub))         # in diagonale non si supera il fondo
                ua, ub = ua / k, ub / k
            else:
                ua, ub = ly, ry                        # slider: un motore ciascuno (mappatura versi)
            self.link.send({"t": "raw", "a": round(ua * span, 3), "b": round(ub * span, 3)})
        else:
            self.link.send({"t": "cmd", "p": round(py * float(b.get("pitch_span", 8.0)), 2),
                            "r": round(rx * float(b.get("roll_span", 6.0)), 2)})

    # ------------------------------------------------------------ calibrazione
    def _capture(self, joint, pt):
        tel, age = self._tel()
        if age > 0.5:
            messagebox.showwarning("Calibrazione", "Nessuna telemetria dal micro.")
            return
        raw = tel.get("pr" if joint == "pitch" else "rr")
        try:
            ang = float(self.cal_entries[(joint, pt)].get())
        except ValueError:
            messagebox.showwarning("Calibrazione", "Angolo non valido.")
            return
        self.capture[(joint, pt)] = (int(raw), ang)
        self.log(f"[banco] {joint} {pt}: raw {raw} = {ang} gradi")
        self.cal_lbl.config(text="  ".join(f"{j}{p}: {r}@{a:+.1f}"
                                           for (j, p), (r, a) in sorted(self.capture.items())))

    def _save_cal(self):
        if not self.role:
            return
        vals = {}
        for j in ("pitch", "roll"):
            if (j, "A") in self.capture and (j, "B") in self.capture:
                (ra, aa), (rb, ab) = self.capture[(j, "A")], self.capture[(j, "B")]
                if ra == rb or aa == ab:
                    messagebox.showwarning("Calibrazione", f"{j}: le due pose sono uguali.")
                    return
                vals.update({f"{j}.raw_min": ra, f"{j}.ang_min": aa,
                             f"{j}.raw_max": rb, f"{j}.ang_max": ab})
        if not vals:
            messagebox.showinfo("Calibrazione", "Cattura A e B di almeno un giunto.")
            return
        self.prof.save_local(self.role, vals)
        self.log(f"[banco] calibrazione salvata in {self.prof.local_path(self.role)}. "
                 "Da RICOPIARE nel repo:")
        for k, v in sorted(vals.items()):
            self.log(f"    {k}: {v}")
        self._push_cfg()

    # ------------------------------------------------------------ OTA
    def _ota(self):
        tel, _ = self._tel()
        if tel.get("st") == "armato":
            messagebox.showwarning("OTA", "Prima disarmare (si aggiorna solo a robot fermo).")
            return
        path = filedialog.askopenfilename(title="firmware .bin",
                                          filetypes=[("firmware", "*.bin"), ("tutti", "*")])
        if not path or not self.link.target_ip:
            return
        d, name = os.path.split(path)
        if self.http is None:
            handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=d)
            socketserver.TCPServer.allow_reuse_address = True
            self.http = socketserver.ThreadingTCPServer(("0.0.0.0", OTA_HTTP_PORT), handler)
            threading.Thread(target=self.http.serve_forever, daemon=True).start()
        else:
            self.http.RequestHandlerClass = functools.partial(
                http.server.SimpleHTTPRequestHandler, directory=d)
        me = local_ip_towards(self.link.target_ip)
        url = f"http://{me}:{OTA_HTTP_PORT}/{name}"
        self.log(f"[banco] OTA: {url}")
        self.link.send({"t": "ota", "url": url})

    # ------------------------------------------------------------ disegno
    def _bar(self, c, span, vals):
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        mid = w / 2
        c.create_line(mid, 0, mid, h, fill=GRID)
        for v, col, y0, y1 in vals:
            if v is None:
                continue
            x = mid + max(-1.0, min(1.0, v / span)) * (mid - 3)
            if col == MEAS_COL:
                c.create_rectangle(min(mid, x), y0, max(mid, x), y1, fill=col, outline="")
            else:
                c.create_line(x, y0, x, y1, fill=col, width=3)

    def _tick(self):
        if self.node is not None:
            # spin_once esegue UNA callback: a giro se ne svuotano fino a JOY_DRAIN, perche' gli
            # stick arrivano a ~2x50 Hz e qui si gira a 20 Hz. Con una sola, le code restavano
            # piene e stick (e fungo) arrivavano con ~1 s di ritardo (19/10).
            for _ in range(JOY_DRAIN):
                rclpy.spin_once(self.node, timeout_sec=0.0)
        self._ticks = getattr(self, "_ticks", 0) + 1
        if self._ticks % 10 == 0:            # 2 Hz: alla lista basta
            self._refresh_micros()
            if self.mac is None:
                self._auto_attach()
        tel, age = self._tel()
        if self.role and self.link.target_ip and not self.cfg_sent and age < 1.0:
            self._push_cfg()
        live = age < 0.5
        st = tel.get("st", "—") if live else "offline"
        f = tel.get("f", "")
        color = {"armato": OK_COL, "off": WARN_COL, "fault": ERR_COL}.get(st, OFF_COL)
        txt = f"{st.upper()}"
        if f:
            txt += f"  ({f})"
        settled = time.time() - getattr(self, "mode_click", 0.0) > 0.5   # telemetria in ritardo sul clic
        if live and settled and tel.get("mode") in ("pos", "raw") and tel["mode"] != self.mode_var.get():
            self.mode_var.set(tel["mode"])     # il pallino segue il micro, non l'ultimo clic
        if live:
            txt += f"   {tel.get('mode', '').upper()}" + ("" if tel.get("cal") else "  NON CALIBRATO")
        self.state_lbl.config(text=txt, bg=color)
        self.arm_btn.config(text="DISARMA" if st == "armato" else "ARMA",
                            bg=ERR_COL if st == "armato" else WARN_COL)
        if live:
            for j, k in (("pitch", "p"), ("roll", "r")):
                c, lbl, span = self.bars[j]
                m, t, s = tel.get(k), tel.get(k + "t"), tel.get(k + "s")
                self._bar(c, span, [(m, MEAS_COL, 4, 22), (t, TGT_COL, 0, 26), (s, SP_COL, 0, 26)])
                lbl.config(text=f"mis {m:+7.2f}  target {t:+7.2f}  set {s:+7.2f}\n"
                                f"vel {tel.get(k + 'v', 0):+6.1f}  u {tel.get('u' + k, 0):+.3f}  "
                                f"raw {tel.get(k + 'r')}")
            for m, k in (("A", "ua"), ("B", "ub")):
                self._bar(self.duty_bars[m], 1.0, [(tel.get(k), MEAS_COL, 2, 16)])
            wd = tel.get("wd", -1)
            self.motor_lbl.config(
                text=f"A duty {tel.get('ua', 0):+.3f}  I {tel.get('ia', 0):+6.2f} A ({tel.get('cma')} mV)\n"
                     f"B duty {tel.get('ub', 0):+.3f}  I {tel.get('ib', 0):+6.2f} A ({tel.get('cmb')} mV)\n"
                     f"saturo {tel.get('sat')}   heartbeat visto dal micro: {wd} ms fa"
                     + ("   ⚠ FUNGO" if self.estop else ""))
        self._send_command(tel if live else {})
        self._func_show()
        self.root.after(50, self._tick)    # 20 Hz di comandi, come l'heartbeat

    def _on_close(self):
        self.link.send({"t": "arm", "on": False})
        self.link.close()
        if self.http:
            self.http.shutdown()
        self.root.destroy()

    def run(self):
        self.root.after(100, self._tick)
        self.root.mainloop()


def main(args=None):
    ap = argparse.ArgumentParser(description="Banco dei micro AIRA")
    ap.add_argument("--repo", default=os.environ.get("AIRA_REPO", "~/AIRA_Robot"),
                    help="clone di AIRA_Robot (profili in periph/)")
    ap.add_argument("--no-ros", action="store_true", help="niente ROS: comandano gli slider")
    ap.add_argument("--windowed", action="store_true", help="in finestra invece che a schermo intero")
    known, ros_args = ap.parse_known_args(args)
    node = None
    if HAVE_ROS and not known.no_ros:
        rclpy.init(args=ros_args)
        node = rclpy.create_node("aira_bench")
    gui = Bench(known, node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
