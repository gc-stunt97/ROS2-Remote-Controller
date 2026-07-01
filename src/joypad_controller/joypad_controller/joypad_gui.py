#!/usr/bin/env python3
"""GUI dei joystick — "plancia 2D".

Ogni joystick e' mostrato come un pad quadrato con un pallino che segue X
(orizzontale) e Y (verticale), con crosshair e cerchio di fondo scala; accanto
una barra verticale per lo Z (yaw), e sotto i valori numerici.

Sottoscrive `left_joystick_data` / `right_joystick_data` (geometry_msgs/Point).
Si puo' provare SENZA robot: basta controller + STM32 collegato (+ joy_node in
esecuzione che pubblica i topic).

Integrazione ROS+Tk: il ciclo lo guida Tkinter (`mainloop`); a ogni tick facciamo
`spin_once` per far girare le callback ROS e poi ridisegniamo.
"""

import tkinter as tk

import rclpy
from geometry_msgs.msg import Point

# --- estetica ---
PAD = 220        # lato del pad quadrato (px)
DOT_R = 9        # raggio del pallino
BAR_W = 30       # larghezza della barra Z
REFRESH_MS = 30  # ~33 Hz di ridisegno

BG = "#1e1e2e"
FG = "#cdd6f4"
GRID = "#45475a"
DOT_LEFT = "#89b4fa"   # blu (joystick sinistro)
DOT_RIGHT = "#f38ba8"  # rosa (joystick destro)
BAR_COL = "#a6e3a1"    # verde (barra Z)


class JoypadGui:
    """Finestra Tkinter che visualizza i due joystick in tempo reale."""

    def __init__(self, node):
        self.node = node
        self.values = {"left": Point(), "right": Point()}

        node.create_subscription(
            Point, "left_joystick_data", lambda m: self._store("left", m), 10)
        node.create_subscription(
            Point, "right_joystick_data", lambda m: self._store("right", m), 10)

        self.root = tk.Tk()
        self.root.title("Joystick — RobotHex controller")
        self.root.configure(bg=BG)

        self.widgets = {}
        self._build_side("LEFT", "left", DOT_LEFT, tk.LEFT)
        self._build_side("RIGHT", "right", DOT_RIGHT, tk.RIGHT)

    # --- costruzione UI ---------------------------------------------------
    def _build_side(self, title, key, dot_color, side):
        frame = tk.Frame(self.root, bg=BG)
        frame.pack(side=side, padx=24, pady=16)

        tk.Label(frame, text=title, bg=BG, fg=FG,
                 font=("TkDefaultFont", 14, "bold")).pack(pady=(0, 8))

        row = tk.Frame(frame, bg=BG)
        row.pack()
        pad = tk.Canvas(row, width=PAD, height=PAD, bg=BG,
                        highlightthickness=1, highlightbackground=GRID)
        pad.pack(side=tk.LEFT)
        bar = tk.Canvas(row, width=BAR_W, height=PAD, bg=BG,
                        highlightthickness=1, highlightbackground=GRID)
        bar.pack(side=tk.LEFT, padx=(10, 0))

        val = tk.Label(frame, text="X:+0.00  Y:+0.00  Z:+0.00", bg=BG, fg=FG,
                       font=("TkFixedFont", 11))
        val.pack(pady=(8, 0))

        self.widgets[key] = {"pad": pad, "bar": bar, "val": val, "color": dot_color}

    # --- callback ROS -----------------------------------------------------
    def _store(self, key, msg):
        self.values[key] = msg

    @staticmethod
    def _clamp(v):
        return max(-1.0, min(1.0, float(v)))

    # --- disegno ----------------------------------------------------------
    def _draw_pad(self, canvas, x, y, color):
        canvas.delete("all")
        c = PAD / 2
        half = PAD / 2 - DOT_R - 2
        canvas.create_line(c, 4, c, PAD - 4, fill=GRID)          # crosshair vert.
        canvas.create_line(4, c, PAD - 4, c, fill=GRID)          # crosshair orizz.
        canvas.create_oval(c - half, c - half, c + half, c + half, outline=GRID)
        px = c + self._clamp(x) * half           # X: destra = +
        py = c - self._clamp(y) * half           # Y: su = + (schermo invertito)
        canvas.create_oval(px - DOT_R, py - DOT_R, px + DOT_R, py + DOT_R,
                           fill=color, outline="")

    def _draw_bar(self, canvas, z):
        canvas.delete("all")
        c = PAD / 2
        top = c - self._clamp(z) * (PAD / 2 - 4)   # zero al centro, riempie col segno
        canvas.create_line(2, c, BAR_W - 2, c, fill=GRID)
        canvas.create_rectangle(4, min(c, top), BAR_W - 4, max(c, top),
                                fill=BAR_COL, outline="")

    # --- ciclo principale -------------------------------------------------
    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for key, w in self.widgets.items():
            m = self.values[key]
            self._draw_pad(w["pad"], m.x, m.y, w["color"])
            self._draw_bar(w["bar"], m.z)
            w["val"].config(text=f"X:{m.x:+.2f}  Y:{m.y:+.2f}  Z:{m.z:+.2f}")
        self.root.after(REFRESH_MS, self._tick)

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
