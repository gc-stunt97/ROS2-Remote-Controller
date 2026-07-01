#!/usr/bin/env python3
"""Nodo publisher dei joystick.

Legge la seriale dello STM32 (Blue Pill), interpreta la riga JSON
{"LX","LY","LZ","RX","RY","RZ","BL","BR"} e pubblica:
- `left_joystick_data` / `right_joystick_data`  (geometry_msgs/Point, assi)
- `left_button` / `right_button`                (std_msgs/Bool, tastino in cima)

Convenzione assi ROS: x = laterale (destra +), y = avanti (+), z = yaw.
I pulsanti sono opzionali: se il firmware non e' ancora aggiornato (chiavi BL/BR
assenti) vengono pubblicati come False, senza errori.

Robustezza:
- porta e baud sono parametri ROS2 (niente valori "murati" nel codice);
- la lettura seriale gira in un thread dedicato (non blocca l'esecutore ROS);
- se l'USB e' scollegato o da' errore, riprova ad aprirlo da solo (riconnessione);
- righe non decodificabili / JSON malformato / chiavi mancanti vengono scartate;
- chiusura pulita (thread fermato, seriale chiusa, no doppio rclpy.shutdown).
"""

import json
import threading

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class JoystickNode(Node):
    """Legge lo STM32 via seriale e ripubblica joystick + tastini su ROS2."""

    def __init__(self):
        super().__init__("joystick_node")

        # --- parametri (sovrascrivibili con --ros-args -p serial_port:=... ) ---
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 57600)
        self.declare_parameter("reconnect_period", 2.0)  # s di attesa tra i tentativi
        # Il firmware ATTUALE sul micro scala lo yaw x2 -> Z arriva a ~±2. Con 0.5 lo
        # riportiamo a ±1 come gli altri assi, SENZA riflashare. (Il firmware aggiornato
        # nel repo ha gia' yaw x1: quando lo flasherai via ST-Link, metti yaw_scale:=1.0.)
        self.declare_parameter("yaw_scale", 0.5)
        self._port = self.get_parameter("serial_port").value
        self._baud = int(self.get_parameter("baud").value)
        self._reconnect = float(self.get_parameter("reconnect_period").value)
        self._yaw_scale = float(self.get_parameter("yaw_scale").value)

        self._pub_left = self.create_publisher(Point, "left_joystick_data", 10)
        self._pub_right = self.create_publisher(Point, "right_joystick_data", 10)
        self._pub_btn_left = self.create_publisher(Bool, "left_button", 10)
        self._pub_btn_right = self.create_publisher(Bool, "right_button", 10)

        self._serial = None
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(
            f"joystick_node avviato (porta={self._port}, baud={self._baud})")

    # --- gestione seriale -------------------------------------------------
    def _open_serial(self):
        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
            self.get_logger().info(f"Seriale aperta su {self._port}")
            return True
        except serial.SerialException as exc:
            self.get_logger().warn(
                f"Seriale non disponibile ({exc}); riprovo tra {self._reconnect}s")
            return False

    def _close_serial(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
            self._serial = None

    def _read_loop(self):
        """Thread: legge righe dalla seriale e pubblica, con riconnessione."""
        while not self._stop.is_set():
            if self._serial is None:
                if not self._open_serial():
                    self._stop.wait(self._reconnect)  # backoff
                    continue
            try:
                raw = self._serial.readline()
            except (serial.SerialException, OSError, TypeError) as exc:
                if self._stop.is_set():
                    break  # chiusura in corso: usciamo in silenzio
                self.get_logger().warn(f"Errore seriale ({exc}); riconnetto")
                self._close_serial()
                continue
            if raw:  # readline vuoto = timeout senza dati, riprova
                self._publish_line(raw)

    def _publish_line(self, raw):
        try:
            line = raw.decode("utf-8").strip()
        except UnicodeDecodeError:
            return
        if not line:
            return
        try:
            data = json.loads(line)
            # Convenzione ROS: x = laterale (destra +), y = avanti (+), z = yaw.
            # Il firmware ha gli assi scambiati (avanti->X, destra->Y): rimappiamo.
            left = Point(x=float(data["LY"]), y=float(data["LX"]),
                         z=float(data["LZ"]) * self._yaw_scale)
            right = Point(x=float(data["RY"]), y=float(data["RX"]),
                          z=float(data["RZ"]) * self._yaw_scale)
            # Pulsanti: opzionali (assenti se il firmware non e' ancora aggiornato).
            btn_left = bool(int(data.get("BL", 0)))
            btn_right = bool(int(data.get("BR", 0)))
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            return  # riga sporca: la saltiamo
        self._pub_left.publish(left)
        self._pub_right.publish(right)
        self._pub_btn_left.publish(Bool(data=btn_left))
        self._pub_btn_right.publish(Bool(data=btn_right))

    # --- ciclo di vita ----------------------------------------------------
    def destroy_node(self):
        self._stop.set()
        self._close_serial()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():         # l'handler SIGINT di rclpy potrebbe aver gia' chiuso
            rclpy.shutdown()


if __name__ == "__main__":
    main()
