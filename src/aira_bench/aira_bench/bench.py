#!/usr/bin/env python3
"""BANCO dei micro AIRA: il telecomando finge di essere il mini PC (CONTROLLER_HANDBOOK §14).

Si attacca UN micro ESP32-P4-ETH alla porta "solo micro" del pannello e questo programma:
  - ascolta gli annunci (hello) e riconosce la scheda dal MAC (tabella `periph/roles.yaml` nel
    clone di AIRA_Robot: niente tendina, e' il micro a presentarsi);
  - si aggancia come CAPO: heartbeat 20 Hz con boss="banco", spinge la config del PROFILO del
    ruolo (`periph/<ruolo>.yaml`, la STESSA che spingera' il mini PC);
  - stick -> giunti (busto: stick SX Y -> pitch, stick DX X -> roll), oppure modo RAW per
    mappare i versi dei motori;
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
from tkinter import filedialog, messagebox, scrolledtext

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

# estetica: come le plance (catppuccin)
BG, FG, GRID, BTN, INK = "#1e1e2e", "#cdd6f4", "#45475a", "#313244", "#11111b"
OK_COL, WARN_COL, ERR_COL, OFF_COL = "#a6e3a1", "#f9e2af", "#f38ba8", "#6c7086"
MEAS_COL, TGT_COL, SP_COL = "#a6e3a1", "#89b4fa", "#f9e2af"


# ---------------------------------------------------------------------- profili
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
        self.hb_enabled = True
        self.on_hello = None      # callback(mac, hello) chiamata dal thread di rete
        self.on_ack = None
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
                    self.micros[mac] = msg
                if self.on_hello:
                    self.on_hello(mac, msg)
            elif ip != self.target_ip:
                continue
            elif t == "tel":
                with self.lock:
                    self.tel = msg
                    self.tel_time = time.time()
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
        self.pending_ip = None     # IP della scheda vista, prima di sapere che ruolo ha
        self.http = None

        self.root = tk.Tk()
        self.root.title("AIRA — banco micro")
        self.root.configure(bg=BG)
        self._build()

        self.link = Link(self.log)
        self.link.on_hello = lambda mac, h: self.root.after(0, self._on_hello, mac, h)
        self.link.on_ack = lambda m: None

        if node is not None:
            node.create_subscription(Point, "left_joystick_data",
                                     lambda m: self._on_joy("left", m), 10)
            node.create_subscription(Point, "right_joystick_data",
                                     lambda m: self._on_joy("right", m), 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(Bool, "/emergency_stop", self._on_estop, latched)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.log(f"[banco] profili da {self.prof.dir}"
                 + ("" if node else "  (SENZA ROS: comandano gli slider)"))
        self.log("[banco] in attesa che un micro si presenti sulla porta "
                 f"{HELLO_PORT}...")

    # ------------------------------------------------------------ costruzione
    def _btn(self, parent, text, cmd, **kw):
        return tk.Button(parent, text=text, command=cmd, bg=BTN, fg=FG, relief=tk.FLAT,
                         activebackground=GRID, **kw)

    def _build(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill=tk.X, padx=8, pady=(6, 2))
        self.who = tk.Label(top, text="nessun micro", bg=BG, fg=FG, anchor="w",
                            font=("TkFixedFont", 10))
        self.who.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.assign_btn = self._btn(top, "usa come BUSTO (solo ora)", self._assign_torso)

        main = tk.Frame(self.root, bg=BG)
        main.pack(fill=tk.BOTH, expand=True, padx=8)

        left = tk.Frame(main, bg=BG)
        left.pack(side=tk.LEFT, anchor="n")
        self.state_lbl = tk.Label(left, text="—", bg=OFF_COL, fg=INK, width=34,
                                  font=("TkDefaultFont", 13, "bold"))
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
        self._btn(row2, "clear fault", lambda: self.link.send({"t": "clear"})).pack(side=tk.LEFT)
        self.hb_btn = self._btn(row2, "stacca heartbeat", self._toggle_hb)
        self.hb_btn.pack(side=tk.LEFT, padx=4)
        self._btn(row2, "rimanda config", self._push_cfg).pack(side=tk.LEFT)
        self._btn(row2, "leggi config", lambda: self.link.send({"t": "get"})).pack(side=tk.LEFT, padx=4)

        # barre giunti
        self.bars = {}
        for j, span in (("pitch", 25.0), ("roll", 20.0)):
            tk.Label(left, text=j.upper(), bg=BG, fg=FG, anchor="w",
                     font=("TkDefaultFont", 10, "bold")).pack(fill=tk.X, pady=(6, 0))
            c = tk.Canvas(left, width=320, height=26, bg=BG, highlightthickness=1,
                          highlightbackground=GRID)
            c.pack()
            lbl = tk.Label(left, text="", bg=BG, fg=FG, font=("TkFixedFont", 9), anchor="w")
            lbl.pack(fill=tk.X)
            self.bars[j] = (c, lbl, span)
        tk.Label(left, text="verde = misurato, blu = target, giallo = setpoint (rampa)",
                 bg=BG, fg=GRID, font=("TkFixedFont", 8)).pack(anchor="w")

        tk.Label(left, text="MOTORI (duty) / CORRENTE", bg=BG, fg=FG, anchor="w",
                 font=("TkDefaultFont", 10, "bold")).pack(fill=tk.X, pady=(8, 0))
        self.duty_bars = {}
        for m in ("A", "B"):
            c = tk.Canvas(left, width=320, height=18, bg=BG, highlightthickness=1,
                          highlightbackground=GRID)
            c.pack(pady=1)
            self.duty_bars[m] = c
        self.motor_lbl = tk.Label(left, text="", bg=BG, fg=FG, font=("TkFixedFont", 9),
                                  anchor="w", justify="left")
        self.motor_lbl.pack(fill=tk.X)

        # slider (senza ROS o senza joystick)
        sl = tk.LabelFrame(left, text="slider (comandano se il joystick tace)", bg=BG, fg=FG)
        sl.pack(fill=tk.X, pady=6)
        self.sl = {}
        for name in ("pitch / mot A", "roll / mot B"):
            s = tk.Scale(sl, from_=-1.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL,
                         length=300, label=name, bg=BG, fg=FG, troughcolor=BTN,
                         highlightthickness=0)
            s.pack()
            self.sl[name] = s
        self._btn(sl, "slider a zero", lambda: [s.set(0) for s in self.sl.values()]).pack(pady=2)

        # destra: calibrazione + OTA + log
        right = tk.Frame(main, bg=BG)
        right.pack(side=tk.LEFT, anchor="n", padx=(12, 0), fill=tk.BOTH, expand=True)
        cal = tk.LabelFrame(right, text="calibrazione pot (due pose, angolo dalla livella)",
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

        ota = tk.Frame(right, bg=BG)
        ota.pack(fill=tk.X, pady=6)
        self._btn(ota, "OTA: scegli .bin", self._ota).pack(side=tk.LEFT)
        self._btn(ota, "riavvia micro", self._reboot).pack(side=tk.LEFT, padx=4)

        self.logbox = scrolledtext.ScrolledText(right, width=70, height=26, bg=INK, fg=FG,
                                                font=("TkFixedFont", 9))
        self.logbox.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------ log (thread-safe)
    def log(self, text):
        line = time.strftime("%H:%M:%S ") + text + "\n"
        try:
            self.root.after(0, self._append_log, line)
        except RuntimeError:
            pass

    def _append_log(self, line):
        self.logbox.insert(tk.END, line)
        if int(self.logbox.index("end-1c").split(".")[0]) > 2000:
            self.logbox.delete("1.0", "500.0")
        self.logbox.see(tk.END)

    # ------------------------------------------------------------ aggancio
    def _on_hello(self, mac, h):
        if self.mac is None:
            role = self.prof.roles().get(mac)
            self.mac = mac
            self.pending_ip = h["ip_src"]
            if role:
                self._attach(role)
            else:
                self.log(f"[banco] scheda SCONOSCIUTA {mac} ({h['ip_src']}): aggiungi "
                         f"'\"{mac}\": torso' a periph/roles.yaml (o usa il pulsante in alto)")
                self.assign_btn.pack(side=tk.RIGHT)
        elif mac == self.mac:
            self.pending_ip = h["ip_src"]
            if self.role:
                self.link.target_ip = h["ip_src"]
            # il micro si e' riavviato o ci ha perso: la config va rispinta
            if self.role and h.get("boss") == "nessuno" and self.link.hb_enabled:
                self.cfg_sent = False
        self.last_hello = h
        self._who(h)

    def _who(self, h):
        role = self.role or "SCONOSCIUTO"
        self.who.config(text=f"{role}  MAC {h.get('mac')}  IP {h.get('ip_src')}  fw {h.get('fw')} "
                             f"[{h.get('part')}, {h.get('img')}]  capo: {h.get('boss')}")

    def _assign_torso(self):
        self.assign_btn.pack_forget()
        self._attach("torso")

    def _attach(self, role):
        try:
            prof = self.prof.profile(role)
        except (OSError, RuntimeError) as exc:
            messagebox.showerror("Profilo", f"Non leggo il profilo '{role}':\n{exc}")
            return
        self.role = role
        self.bench_cfg = prof.get("bench") or {}
        # solo ORA si diventa capo: a una scheda senza ruolo non si manda nemmeno l'heartbeat
        self.link.target_ip = self.pending_ip
        self.log(f"[banco] agganciato {self.mac} come '{role}'")
        self.cfg_sent = False
        if getattr(self, "last_hello", None):
            self._who(self.last_hello)

    def _push_cfg(self):
        if not self.role:
            return
        cfg = self.prof.cfg(self.role)
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
        mode = tel.get("mode", "pos")
        if not messagebox.askyesno("Arma", f"Armare il busto in modo {mode.upper()}?\n"
                                   "Mani lontane da leve e tiranti."):
            return
        self.link.send({"t": "arm", "on": True})

    def _set_mode(self):
        self.link.send({"t": "mode", "m": self.mode_var.get()})

    def _toggle_hb(self):
        self.link.hb_enabled = not self.link.hb_enabled
        if self.link.hb_enabled:
            self.hb_btn.config(text="stacca heartbeat", bg=BTN, fg=FG)
            self.log("[banco] heartbeat RIATTACCATO")
        else:
            self.hb_btn.config(text="RIATTACCA heartbeat", bg=ERR_COL, fg=INK)
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
        if tel.get("st") != "armato" or self.estop:
            return
        joy_live = self.node is not None and time.time() - self.joy_time < JOY_TIMEOUT
        if joy_live:
            lx, ly, _ = self.joy["left"]
            rx, ry, _ = self.joy["right"]
        else:
            ly = self.sl["pitch / mot A"].get()
            rx = ry = self.sl["roll / mot B"].get()
        b = getattr(self, "bench_cfg", {})
        if tel.get("mode") == "raw":
            span = float(b.get("raw_span", 0.3))
            self.link.send({"t": "raw", "a": round(ly * span, 3), "b": round(ry * span, 3)})
        else:
            self.link.send({"t": "cmd", "p": round(ly * float(b.get("pitch_span", 8.0)), 2),
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
            rclpy.spin_once(self.node, timeout_sec=0.0)
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
                lbl.config(text=f"mis {m:+7.2f}  target {t:+7.2f}  set {s:+7.2f}  "
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
