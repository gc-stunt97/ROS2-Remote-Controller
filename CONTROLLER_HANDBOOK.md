# ROS2 Remote Controller — Handbook & contesto

> Controller remoto a joystick per l'esapode **RobotHex** (e riutilizzabile per altri
> robot ROS2). Questo documento è la **fonte di verità** del progetto controller:
> hardware, architettura, come si avvia, firmware, problemi noti, roadmap.
> Da leggere all'inizio di ogni ripresa. Compagno del `ROBOTHEX_HANDBOOK.md` (lato robot).
>
> 📘 **Per il quadro d'insieme del SISTEMA** (architettura ROS di robot+controller, come si
> usa, comandi ROS, troubleshooting) → `MANUALE.md` nel repo **RobotHex**. Questo file resta
> per i dettagli specifici del controller.

---

## 0. Stato attuale — RIPRENDI DA QUI (luglio 2026)

**Fatto e funzionante:**
- Catena completa: STM32 → seriale → nodo ROS2 → topic → GUI, **verificata sul 7"**.
- **GUI "plancia 2D"** (pad X/Y + barra Z + valori), assi rimappati alla convenzione ROS.
- Avvio **user-friendly**: icona sul desktop → parte joystick + GUI insieme.
- **Backup git** su GitHub privato: `git@github.com:gc-stunt97/ROS2-Remote-Controller.git`.

**In sospeso (parcheggiato):**
- **Flash del firmware STM32** con le migliorie (tastini `BL/BR`, yaw ×1): bloccato dai
  tool DFU su ARM64 → si farà con **ST-Link/SWD** (vedi sez. 6). Finché non si flasha:
  - i **tastini** dei joystick non sono attivi;
  - lo **yaw** è normalizzato a ±1 lato Pi con `yaw_scale=0.5` (vedi sez. 4).

**Prossimi passi:** (a) flashare il firmware via ST-Link; (b) **integrazione col robot**:
un nodo *teleop* che traduce i joystick in comandi di andatura (velocità/direzione/rotazione
→ parametri del gait engine del robot).

---

## 1. Cos'è

Controller remoto **autocostruito** (replica del design di **James Bruton**) per pilotare
il robot via WiFi usando ROS2. È il "telecomando intelligente": legge due joystick, li
pubblica su topic ROS2, e il robot (subscriber sulla stessa rete) li usa per muoversi.

---

## 2. Hardware

- **Raspberry Pi 4** = cervello del controller. OS: **Ubuntu 22.04 "jammy" arm64**,
  ROS2 **humble**.
- **Display 7"** (touchscreen) collegato al Pi → mostra la GUI dei joystick.
- **2 joystick arcade a 3 assi**: X e Y classici + **Z** ruotando la testa (yaw).
- **STM32 "Blue Pill"** (STM32F103C8): legge i 6 assi analogici + i pulsanti e manda i
  valori al Pi via **seriale USB** (`/dev/ttyACM0`).

### Alimentazione
- Oggi: **powerbank ~10000 mAh** via **USB 5 V** → alimenta Pi 4 + display 7" + STM32 +
  joystick. La **USB-C** della powerbank è riservata a **ricarica / pass-through** (si può
  tenere in carica mentre si usa), quindi non la si usa per alimentare altro.
- **Autonomia** stimata ~2 h col solo controller (di meno col RUT241 attivo).
- **Roadmap — link dedicato (RUT241 Teltonika):** router industriale (RutOS/OpenWrt, WiFi
  2.4 GHz + LTE) da integrare nel controller come **hub di rete privata** robot↔controller.
  Architettura prevista: **Pi controller cablato in Ethernet** al RUT241 (WiFi dedicata al
  solo robot), 4G come **backhaul** opzionale. Alimentazione RUT241 (vuole ~9–50 V, non 5 V):
  **boost DC-DC 5 V → 12 V** dalla powerbank. Sta **dopo il brownout** nelle priorità.

---

## 3. Architettura software

```
STM32 (Blue Pill)  --- legge 6 assi + 2 tastini
      │  applica deadzone + scaling → riga JSON
      │  Serial.println @ 57600 baud  (USB /dev/ttyACM0, ~50 Hz)
      ▼
joy_node  (nodo ROS2 "joystick_node")
      │  json.loads, rimappa assi, pubblica
      ├── /left_joystick_data   (geometry_msgs/Point)   assi joystick sinistro
      ├── /right_joystick_data  (geometry_msgs/Point)   assi joystick destro
      ├── /left_button          (std_msgs/Bool)         tastino sinistro
      └── /right_button         (std_msgs/Bool)         tastino destro
      ▼
joypad_gui  (GUI "plancia 2D" sul 7")  ← e, in futuro, il ROBOT (subscriber via WiFi)
```

**Convenzione assi (decisa qui, valida end-to-end):**
`x` = laterale (destra +), `y` = avanti (+), `z` = yaw/rotazione. Valori ~ **[-1, 1]**.

---

## 4. Pacchetto ROS2 `joypad_controller`

Workspace sul Pi: `~/ros2_ws/`, pacchetto `src/joypad_controller/` (ament_python).

- **`joy_node.py`** → nodo publisher. Legge la seriale in un **thread dedicato** con
  **riconnessione automatica** (se stacchi/riattacchi l'USB non crasha), scarta le righe
  sporche, chiusura pulita. Parametri ROS2:
  - `serial_port` (default `/dev/ttyACM0`)
  - `baud` (default `57600`)
  - `reconnect_period` (default `2.0` s)
  - **`yaw_scale`** (default `0.5`) → il firmware *attuale* scala lo yaw ×2 (Z ~±2);
    0.5 lo riporta a ±1. **Dopo aver flashato il firmware nuovo (yaw ×1), mettere `1.0`.**
- **`joypad_gui.py`** → GUI Tkinter: ogni joystick è un **pad 2D** con un pallino (X/Y),
  crosshair e cerchio di fondo scala; accanto una **barra** per lo Z; sotto i valori.
  Il pallino è **vuoto a riposo** e si **riempie** mentre premi il tastino (quando il
  firmware invierà `BL/BR`).
- **`joypad.launch.py`** → avvia `joy_node` + GUI insieme. Chiudendo la GUI si spegne
  anche il nodo.
- Eseguibili ROS2: **`joypad_node`** e **`joypad_gui_app`**.

---

## 5. Come si avvia

### Modo user-friendly (consigliato)
Sul desktop del controller c'è l'icona **"RobotHex Controller"** → doppio-click, parte
tutto sul 7". (Setup dell'icona: vedi `desktop/README.md` nel repo.)

### Con ros2 launch
```bash
ros2 launch joypad_controller joypad.launch.py
```
⚠️ La GUI ha bisogno di uno schermo: lanciala **dal display del controller**, oppure da
SSH con `export DISPLAY=:0` prima del comando (lo script `~/robothex-controller.sh` lo fa
già da solo).

### Nodi separati (debug)
```bash
ros2 run joypad_controller joypad_node        # solo publisher (va bene da SSH)
ros2 run joypad_controller joypad_gui_app     # GUI (serve display)
```

### Ispezione / test senza toccare la GUI
```bash
ros2 topic echo /right_joystick_data          # vedi i dati che passano
ros2 topic echo /left_button                  # stato tastino (quando attivo)
ros2 topic pub /right_joystick_data geometry_msgs/msg/Point "{x: 0.0, y: 1.0, z: 0.0}"
```

---

## 6. Firmware STM32

- **Sorgente:** `firmware/stm32/ROS2controller/ROS2controller.ino` (Arduino/STM32duino,
  libreria **ArduinoJson**). Legge PA0–PA5 (assi) + PB0/PB1 (tastini), applica deadzone e
  scaling, e stampa una riga JSON. Dettagli protocollo: `firmware/stm32/README.md`.
- **Bootloader della scheda:** **Maple** (USB id `1eaf:0004` in esecuzione, `1eaf:0003`
  in DFU). La scheda si programma **via USB**, non serve l'ST-Link… in teoria (vedi sotto).

### ⚠️ Come flashare (stato attuale) — usare l'ST-Link
Il flash via USB/DFU **dal Pi (ARM64)** al momento **non funziona**: il core STM32 non
include i binari `upload_reset`/`dfu-util` per aarch64, e col reset fisico la scheda non
resta in DFU abbastanza. Da **Windows** entra in DFU ma manca il driver (servirebbe Zadig).
→ **La via pulita e definitiva è l'ST-Link via SWD** (4 fili: SWDIO, SWCLK, GND, 3V3),
che parla direttamente al chip ignorando USB e bootloader. Su Linux: `stlink-tools` / `st-flash`.

Compilazione sul Pi (funziona già, produce il `.bin`):
```bash
arduino-cli compile -b STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 \
  --output-dir ~/fw_out firmware/stm32/ROS2controller
```

---

## 7. Problemi noti / gotcha (imparati sul campo)

- **Flash DFU su Raspberry (aarch64):** non va (tool mancanti). Usare **ST-Link**.
- **La GUI da SSH** dà `no $DISPLAY`: serve `export DISPLAY=:0` (o lanciarla dal 7", o via
  lo script/icona che lo gestiscono).
- **`yaw_scale=0.5`** è un tampone finché il firmware ha lo yaw ×2. Dopo il flash → `1.0`.
- **Tastini** (`BL/BR`) inattivi finché non si flasha il firmware aggiornato.
- **USB del Pi si "impunta"** dopo tanti reset/flash falliti (la scheda sparisce da
  `lsusb`): **NON è rotta** — si risolve con `sudo reboot` del Pi.
- **Blue Pill:** connettore micro-USB fragile, maneggiare i cavi con delicatezza.
- **`unattended-upgrades`** (auto-updater di Ubuntu) può riempire `/boot/firmware` (piccola)
  e bloccare `apt`: se ricapita, liberare i `.bak` in `/boot/firmware` e riavviare. Si può
  disattivare l'updater per trattare il controller come appliance.

---

## 8. Workflow di sviluppo

Come per il robot, **GitHub fa da ponte** (Claude non è installato sul Pi):
```
Windows (clone del repo)  --edit-->  git push  --->  GitHub
                                                        │
Raspberry controller  <--- git pull <-------------------┘
      └── colcon build --symlink-install --packages-select joypad_controller
      └── source install/setup.bash   (o l'icona/script lo fa da sé)
```
Con `--symlink-install` le modifiche ai file **Python** non richiedono rebuild; il rebuild
serve quando cambiano `setup.py`/`package.xml`/launch/dipendenze.

---

## 9. Roadmap

1. **Flash firmware via ST-Link** → attiva i tastini + yaw ×1 (poi `yaw_scale:=1.0`).
2. **Integrazione col robot (il pezzo grosso):** nodo *teleop* che sottoscrive
   `right_joystick_data` (avanti = velocità, yaw = rotazione) e `left_joystick_data`, e
   traduce in **parametri del gait engine** del robot (vedi `ROBOTHEX_HANDBOOK.md`).
3. **Rete:** far parlare controller e robot via WiFi (stesso `ROS_DOMAIN_ID`, DDS discovery).
4. (Poi) streaming video FPV su pipeline dedicata — vedi handbook robot sez. 6b.
