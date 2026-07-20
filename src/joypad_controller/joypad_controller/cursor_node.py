#!/usr/bin/env python3
"""Nodo "modo mouse": uno stick del telecomando diventa il mouse di sistema.

Sottoscrive i topic che pubblica gia' `joy_node` e li traduce nei movimenti di un
**mouse virtuale** creato via uinput:
- `<stick>_joystick_data` (Point): x/y -> cursore, z (yaw) -> rotella
- `<stick>_button`        (Bool) : tastino in cima allo stick -> click (vedi sotto)
- `button_1/2/3`          (Bool) : -> TASTI DI TASTIERA (default: su / invio / giu')

I TRE GENERAL PURPOSE = FRECCE + INVIO, ma SOLO QUI (2026-07-20):
Il telecomando non ha tastiera: con freccia su/giu' e invio si naviga la
cronologia di un terminale aperto sul 7", che e' il caso d'uso che rende il
modo mouse davvero autosufficiente. La mappatura vive **in questo nodo**, che
gira solo FUORI dalle plance: dentro le plance i tre pulsanti restano
**liberi e non assegnati**, da dedicare a funzioni del robot quando si
decideranno. Sono parametri (`button_N_key`, nomi `KEY_*` di evdev): se un
pulsante risulta nel posto sbagliato si scambia il default, senza toccare la
logica. `""` disattiva quel pulsante.

IL TASTINO DELLO STICK, come il touch di un telefono:
- tocchi e rilasci            -> click SINISTRO
- tieni premuto e MUOVI       -> trascinamento (drag)
- tieni premuto e STAI FERMO  -> click DESTRO   (long press, default 0.5 s)
Il prezzo, inevitabile: per capire quale dei tre e', il click sinistro deve
partire al RILASCIO e non alla pressione -- esattamente come sul telefono, dove
infatti non se ne accorge nessuno. Con `long_press_time:=0.0` si torna al
comportamento diretto (click alla pressione, niente tasto destro dal tastino).

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
        # I 3 general purpose -> tasti di tastiera. Nomi evdev, "" = disattivato.
        self.declare_parameter("button_1_key", "KEY_UP")
        self.declare_parameter("button_2_key", "KEY_ENTER")
        self.declare_parameter("button_3_key", "KEY_DOWN")
        self.declare_parameter("key_repeat_delay", 500)    # ms prima di ripetere (0 = mai)
        self.declare_parameter("key_repeat_period", 120)   # ms fra una ripetizione e l'altra
        # Tenuta ferma oltre questo tempo = click destro. 0 = disattiva (click
        # diretto alla pressione, come prima).
        self.declare_parameter("long_press_time", 0.5)

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
        self._long_press_time = float(g("long_press_time").value)
        self._rep_delay = int(g("key_repeat_delay").value)
        self._rep_period = int(g("key_repeat_period").value)

        if self._stick not in ("right", "left"):
            raise SystemExit(f"stick deve essere 'right' o 'left', non '{self._stick}'")

        # topic dei general purpose -> codice tasto (risolto dal nome evdev)
        self._key_map = {}
        for n in (1, 2, 3):
            name = str(g(f"button_{n}_key").value).strip()
            if not name:
                continue
            code = ecodes.ecodes.get(name)
            if code is None:
                raise SystemExit(
                    f"button_{n}_key: '{name}' non e' un tasto evdev noto "
                    f"(esempi: KEY_UP, KEY_DOWN, KEY_ENTER, KEY_TAB, KEY_ESC)")
            self._key_map[f"button_{n}"] = code

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

        # --- tastiera virtuale: un SECONDO device, non gli stessi caps -------
        # I tasti non si aggiungono al device del mouse apposta: libinput
        # classifica un device dalle sue capacita', e un puntatore che dichiara
        # anche KEY_UP e' un ibrido che alcune configurazioni trattano male.
        # Due device distinti = un mouse e una tastiera, entrambi indiscutibili.
        # EV_REP fa fare l'AUTOREPEAT al kernel finche' il pulsante e' giu':
        # tenendo premuto scorri la cronologia invece di battere N volte.
        self._kb = None
        if self._key_map:
            kb_caps = {ecodes.EV_KEY: sorted(set(self._key_map.values()))}
            if self._rep_delay > 0:
                kb_caps[ecodes.EV_REP] = [ecodes.REP_DELAY, ecodes.REP_PERIOD]
            self._kb = UInput(kb_caps, name="AIRA controller keyboard", version=0x1)
            if self._rep_delay > 0:
                self._kb.write(ecodes.EV_REP, ecodes.REP_DELAY, self._rep_delay)
                self._kb.write(ecodes.EV_REP, ecodes.REP_PERIOD, self._rep_period)
                self._kb.syn()

        # --- stato ------------------------------------------------------------
        self._x = self._y = self._z = 0.0
        self._acc_x = self._acc_y = self._acc_scroll = 0.0
        self._keys_down = {}          # code -> premuto?  (per non lasciare tasti giu')
        self._btn_raw = False         # ultimo stato grezzo del tastino stick
        self._key_raw = {}            # topic -> ultimo stato grezzo (per i fronti)
        self._keys_kb_down = {}       # code tastiera -> premuto?
        self._press_state = "idle"    # idle | pending | drag | long
        self._press_since = 0.0
        self._freeze_until = 0.0
        now = time.monotonic()
        self._last_msg = 0.0      # 0 = mai ricevuto: il watchdog parte "fermo"
        self._last_tick = now

        # --- sottoscrizioni ---------------------------------------------------
        self.create_subscription(
            Point, f"{self._stick}_joystick_data", self._on_axes, 10)
        self.create_subscription(
            Bool, f"{self._stick}_button", self._on_left_click, 10)
        for topic, code in self._key_map.items():
            # default=code: senza, la lambda catturerebbe l'ultimo code del ciclo.
            self.create_subscription(
                Bool, topic,
                lambda msg, t=topic, c=code: self._on_key(t, c, msg), 10)

        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"cursor_node avviato: stick={self._stick}, speed={self._speed} px/s, "
            f"deadzone={self._deadzone}, expo={self._expo}, rate={self._rate} Hz, "
            + (f"long press {self._long_press_time}s -> tasto destro"
               if self._long_press_time > 0 else "long press disattivato")
            + (", tasti: " + ", ".join(
                f"{t}->{ecodes.KEY[c]}" for t, c in sorted(self._key_map.items()))
               if self._key_map else ", nessun tasto di tastiera")
        )

    # --- callback -------------------------------------------------------------
    def _on_axes(self, msg: Point):
        self._x, self._y, self._z = msg.x, msg.y, msg.z
        self._last_msg = time.monotonic()

    def _on_left_click(self, msg: Bool):
        """Tastino dello stick: click / drag / tasto destro, come il touch."""
        pressed = bool(msg.data)
        if pressed == self._btn_raw:
            return                  # il topic ripubblica a ~50 Hz: solo i fronti
        self._btn_raw = pressed
        now = time.monotonic()
        # Il tastino sta in cima allo stick: premendolo lo stick si sposta sempre
        # un pelo. Congelare il cursore attorno al click evita che il puntatore
        # scivoli via proprio mentre stai cliccando.
        self._freeze_until = now + self._click_freeze

        if pressed:
            if self._long_press_time <= 0.0:
                self._emit_key(ecodes.BTN_LEFT, True)   # modo diretto
                self._press_state = "drag"
            else:
                self._press_state = "pending"           # decide _update_press()
                self._press_since = now
            return

        # --- rilascio ---
        if self._press_state == "pending":
            # Rilasciato prima di decidere: era un tocco -> click sinistro.
            self._emit_key(ecodes.BTN_LEFT, True)
            self._emit_key(ecodes.BTN_LEFT, False)
        elif self._press_state == "drag":
            self._emit_key(ecodes.BTN_LEFT, False)      # fine trascinamento
        # "long": il click destro e' gia' partito, al rilascio non c'e' altro da fare.
        self._press_state = "idle"

    def _on_key(self, topic: str, code: int, msg: Bool):
        """General purpose -> tasto di tastiera, sul fronte (il topic e' a 50 Hz).

        Si tiene giu' finche' il pulsante e' giu' (cosi' vale l'autorepeat del
        kernel), quindi il rilascio DEVE arrivare: se muore la seriale ci pensa
        il watchdog in _tick.
        """
        pressed = bool(msg.data)
        if pressed == self._key_raw.get(topic, False):
            return
        self._key_raw[topic] = pressed
        self._emit_kb(code, pressed)

    def _emit_key(self, code: int, pressed: bool):
        self._ui.write(ecodes.EV_KEY, code, 1 if pressed else 0)
        self._ui.syn()
        self._keys_down[code] = pressed

    def _emit_kb(self, code: int, pressed: bool):
        if self._kb is None:
            return
        self._kb.write(ecodes.EV_KEY, code, 1 if pressed else 0)
        self._kb.syn()
        self._keys_kb_down[code] = pressed

    def _release_all(self):
        """Rilascia tutto: non lasciare mai un tasto premuto per sempre."""
        for code, down in list(self._keys_down.items()):
            if down:
                self._emit_key(code, False)
        for code, down in list(self._keys_kb_down.items()):
            if down:
                self._emit_kb(code, False)
        self._press_state = "idle"
        self._btn_raw = False
        self._key_raw.clear()

    def _update_press(self, now: float):
        """Decide cosa sta diventando una pressione in corso: drag o tasto destro."""
        if self._press_state != "pending":
            return
        # ⚠️ Durante il freeze NON si giudica il movimento: premendo il tastino lo
        #    stick si sposta sempre un po', e senza questa attesa ogni long press
        #    verrebbe scambiato per l'inizio di un trascinamento.
        if now >= self._freeze_until:
            moving = (self._shape(self._x, self._deadzone, self._expo) != 0.0
                      or self._shape(self._y, self._deadzone, self._expo) != 0.0)
            if moving:
                self._emit_key(ecodes.BTN_LEFT, True)   # ti muovi: e' un drag
                self._press_state = "drag"
                return
        if (now - self._press_since) >= self._long_press_time:
            # Fermo abbastanza a lungo: tasto destro, e il rilascio non fara' nulla.
            self._emit_key(ecodes.BTN_RIGHT, True)
            self._emit_key(ecodes.BTN_RIGHT, False)
            self._press_state = "long"

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
        # Rilascia anche i tasti: se la seriale muore mentre stai trascinando, un
        # tasto premuto per sempre bloccherebbe il desktop.
        if self._last_msg == 0.0 or (now - self._last_msg) > self._timeout:
            self._acc_x = self._acc_y = self._acc_scroll = 0.0
            self._release_all()
            return

        self._update_press(now)

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
            self._release_all()   # non lasciare tasti premuti se muoio a meta' click
            self._ui.close()
            if self._kb is not None:
                self._kb.close()
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
