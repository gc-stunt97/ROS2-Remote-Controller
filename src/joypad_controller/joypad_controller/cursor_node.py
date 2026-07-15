#!/usr/bin/env python3
"""Nodo "modo mouse": uno stick del telecomando diventa il mouse di sistema.

Sottoscrive i topic che pubblica gia' `joy_node` e li traduce nei movimenti di un
**mouse virtuale** creato via uinput:
- `<stick>_joystick_data` (Point): x/y -> cursore, z (yaw) -> rotella
- `<stick>_button`        (Bool) : tastino in cima allo stick -> click SINISTRO
- `button_1`              (Bool) : -> click DESTRO (si disattiva con right_click_topic:="")

Perche' uinput e non "simulare i click dentro la GUI": il dispositivo lo crea il
**kernel**, quindi lo vedono TUTTE le applicazioni (desktop, RViz, browser, la
dashboard) senza che nessuna di loro debba saperne niente, e senza toccarne il
codice. Convive col touchscreen: sono due dispositivi di input distinti che
muovono lo stesso cursore.

USO PREVISTO: gira quando NON c'e' una plancia aperta (per navigare Ubuntu sul
7"). Dentro le plance gli stick tornano a comandare il robot e l'interfaccia si
tocca col dito. Il cambio e' automatico e sta negli script di avvio
(`desktop/*.sh`): si passano il testimone con `pkill`, perche' la seriale
`/dev/aira_controller` ha **un solo proprietario alla volta** -- lo stesso motivo
per cui non si lanciano due plance insieme.

PREREQUISITI sul Pi (una tantum, vedi udev/99-aira-uinput.rules):
    sudo apt install python3-evdev
    sudo cp udev/99-aira-uinput.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    sudo usermod -aG input $USER      # poi RILOGGARSI: i gruppi si leggono al login

NOTE DI PROGETTO (le tre cose che non sono ovvie):
- Il movimento uinput e' **relativo e intero**: a bassa velocita' `int(0.4)` = 0 e
  il cursore non si muoverebbe MAI, per quanto tu spinga piano. Si accumula il
  resto frazionario (`_acc_*`) e si emette il pixel quando matura.
- Il cursore si integra a **rate fisso** dal timer, non a ogni messaggio ricevuto:
  cosi' la velocita' non dipende dal ritmo con cui arriva la seriale (~50 Hz), e
  se la seriale singhiozza il cursore rallenta invece di scattare.
- **Watchdog**: se i dati dello stick smettono di arrivare il cursore si ferma.
  Senza, resterebbe l'ultimo valore ricevuto e il cursore scapperebbe da solo
  verso un bordo -- con lo stick fermo in mano.
- Asse Y: sullo schermo Y cresce verso il **basso**, sullo stick avanti e' **Y+**.
  Quindi "avanti = su" richiede il segno meno (parametro `invert_y`, default True).
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

try:
    from evdev import UInput, ecodes
except ImportError as exc:  # dipendenza di sistema, non pip: messaggio esplicito
    raise SystemExit(
        "Manca python3-evdev.  Installa con:  sudo apt install python3-evdev"
    ) from exc


class CursorNode(Node):
    """Traduce uno stick in movimenti di un mouse virtuale uinput."""

    def __init__(self):
        super().__init__("cursor_node")

        # --- parametri (sovrascrivibili con --ros-args -p speed:=1200.0 ...) ---
        self.declare_parameter("stick", "right")          # "right" | "left"
        self.declare_parameter("deadzone", 0.15)          # oltre la quale ci si muove
        self.declare_parameter("speed", 900.0)            # px/s a fondo corsa
        self.declare_parameter("expo", 2.0)               # 1=lineare, 2=preciso al centro
        self.declare_parameter("rate", 90.0)              # Hz di integrazione
        self.declare_parameter("invert_y", True)          # avanti = su
        self.declare_parameter("scroll_deadzone", 0.40)   # lo yaw si sfiora per sbaglio
        self.declare_parameter("scroll_speed", 8.0)       # scatti/s a fondo corsa
        self.declare_parameter("invert_scroll", False)
        self.declare_parameter("click_freeze", 0.12)      # s di cursore fermo dopo un click
        self.declare_parameter("data_timeout", 0.5)       # s senza dati -> stop
        self.declare_parameter("right_click_topic", "button_1")  # "" = disattivato

        g = self.get_parameter
        self._stick = str(g("stick").value)
        self._deadzone = float(g("deadzone").value)
        self._speed = float(g("speed").value)
        self._expo = float(g("expo").value)
        self._rate = float(g("rate").value)
        self._invert_y = bool(g("invert_y").value)
        self._scroll_dz = float(g("scroll_deadzone").value)
        self._scroll_speed = float(g("scroll_speed").value)
        self._invert_scroll = bool(g("invert_scroll").value)
        self._click_freeze = float(g("click_freeze").value)
        self._timeout = float(g("data_timeout").value)
        right_click_topic = str(g("right_click_topic").value)

        if self._stick not in ("right", "left"):
            raise SystemExit(f"stick deve essere 'right' o 'left', non '{self._stick}'")

        # --- mouse virtuale ---------------------------------------------------
        caps = {
            ecodes.EV_REL: [ecodes.REL_X, ecodes.REL_Y, ecodes.REL_WHEEL],
            ecodes.EV_KEY: [ecodes.BTN_LEFT, ecodes.BTN_RIGHT, ecodes.BTN_MIDDLE],
        }
        try:
            self._ui = UInput(caps, name="AIRA controller mouse", version=0x1)
        except PermissionError as exc:
            raise SystemExit(
                "Permesso negato su /dev/uinput (di default e' solo root).\n"
                "Serve la regola udev + il gruppo 'input':\n"
                "  sudo cp udev/99-aira-uinput.rules /etc/udev/rules.d/\n"
                "  sudo udevadm control --reload-rules && sudo udevadm trigger\n"
                "  sudo usermod -aG input $USER      # poi RILOGGATI"
            ) from exc
        except FileNotFoundError as exc:
            raise SystemExit(
                "/dev/uinput non esiste: manca il modulo del kernel.\n"
                "  sudo modprobe uinput   (per renderlo permanente: echo uinput | "
                "sudo tee /etc/modules-load.d/uinput.conf)"
            ) from exc

        # --- stato ------------------------------------------------------------
        self._x = self._y = self._z = 0.0
        self._acc_x = self._acc_y = self._acc_scroll = 0.0
        self._btn_state = {}
        self._freeze_until = 0.0
        now = time.monotonic()
        self._last_msg = 0.0      # 0 = mai ricevuto: il watchdog parte "fermo"
        self._last_tick = now

        # --- sottoscrizioni ---------------------------------------------------
        self.create_subscription(
            Point, f"{self._stick}_joystick_data", self._on_axes, 10)
        self.create_subscription(
            Bool, f"{self._stick}_button", self._on_left_click, 10)
        if right_click_topic:
            self.create_subscription(
                Bool, right_click_topic, self._on_right_click, 10)

        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"cursor_node avviato: stick={self._stick}, speed={self._speed} px/s, "
            f"deadzone={self._deadzone}, expo={self._expo}, rate={self._rate} Hz"
            + (f", click destro da '{right_click_topic}'" if right_click_topic
               else ", click destro disattivato")
        )

    # --- callback -------------------------------------------------------------
    def _on_axes(self, msg: Point):
        self._x, self._y, self._z = msg.x, msg.y, msg.z
        self._last_msg = time.monotonic()

    def _on_left_click(self, msg: Bool):
        self._set_button(ecodes.BTN_LEFT, bool(msg.data))

    def _on_right_click(self, msg: Bool):
        self._set_button(ecodes.BTN_RIGHT, bool(msg.data))

    def _set_button(self, code: int, pressed: bool):
        """Emette il click solo sui FRONTI (il topic ripubblica a ~50 Hz)."""
        if self._btn_state.get(code) == pressed:
            return
        self._btn_state[code] = pressed
        # Il tastino sta in cima allo stick: premendolo lo stick si sposta sempre
        # un pelo. Congelare il cursore attorno al click evita che il puntatore
        # scivoli via proprio mentre stai cliccando.
        self._freeze_until = time.monotonic() + self._click_freeze
        self._ui.write(ecodes.EV_KEY, code, 1 if pressed else 0)
        self._ui.syn()

    # --- curva di risposta ----------------------------------------------------
    @staticmethod
    def _shape(value: float, deadzone: float, expo: float) -> float:
        """Deadzone + curva. Rinormalizza oltre la deadzone: cosi' appena la superi
        parti da zero e non con uno scatto."""
        amount = abs(value)
        if amount <= deadzone:
            return 0.0
        scaled = min((amount - deadzone) / (1.0 - deadzone), 1.0) ** expo
        return math.copysign(scaled, value)

    # --- integrazione ---------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0.0 or dt > 0.25:
            return  # primo giro, o sospensione/salto: non integrare un dt assurdo

        # Watchdog: niente dati freschi -> fermo tutto e azzera gli accumuli.
        if self._last_msg == 0.0 or (now - self._last_msg) > self._timeout:
            self._acc_x = self._acc_y = self._acc_scroll = 0.0
            return

        if now < self._freeze_until:
            return

        moved = False

        vx = self._shape(self._x, self._deadzone, self._expo) * self._speed
        vy = self._shape(self._y, self._deadzone, self._expo) * self._speed
        if self._invert_y:
            vy = -vy
        self._acc_x += vx * dt
        self._acc_y += vy * dt
        dx = int(self._acc_x)
        dy = int(self._acc_y)
        self._acc_x -= dx
        self._acc_y -= dy
        if dx:
            self._ui.write(ecodes.EV_REL, ecodes.REL_X, dx)
            moved = True
        if dy:
            self._ui.write(ecodes.EV_REL, ecodes.REL_Y, dy)
            moved = True

        # Rotella dallo yaw. REL_WHEEL positivo = scroll verso l'alto.
        vs = self._shape(self._z, self._scroll_dz, 1.0) * self._scroll_speed
        if self._invert_scroll:
            vs = -vs
        self._acc_scroll += vs * dt
        ds = int(self._acc_scroll)
        self._acc_scroll -= ds
        if ds:
            self._ui.write(ecodes.EV_REL, ecodes.REL_WHEEL, ds)
            moved = True

        if moved:
            self._ui.syn()

    # --- ciclo di vita --------------------------------------------------------
    def destroy_node(self):
        try:
            # Non lasciare tasti premuti se il nodo muore a meta' click.
            for code, pressed in self._btn_state.items():
                if pressed:
                    self._ui.write(ecodes.EV_KEY, code, 0)
            self._ui.syn()
            self._ui.close()
        except (OSError, AttributeError):
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CursorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
