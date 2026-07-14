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

## 0. Stato attuale — RIPRENDI DA QUI (2026-07-14)

**Controller ricostruito da zero** (nuovo case stampato 3D con alloggiamento per il router
**RUT241 a bordo**), ricablato completamente, STM32 riflashato.

**Fatto in questa sessione (2026-07-14):**
- **Firmware STM32 aggiornato e FLASHATO via ST-Link/SWD** (finalmente!): nuova pinout,
  6 assi + **6 pulsanti** (tastini stick `BL/BR` + **EM STOP** `EM` + 3 general purpose `B1/B2/B3`),
  tutti con pull-up software + **debounce 30 ms**. Yaw ×1.
- **Raddrizzata l'inversione storica X/Y:** nel vecchio firmware `LX/LY` (e `RX/RY`) erano
  **scambiati** e lo swap era compensato in `joy_node.py`. Ora le chiavi JSON sono corrette
  alla sorgente e il nodo fa **pass-through pulito** (`x<-LX`, `y<-LY`, `z<-LZ`). L'uscita ROS
  è identica a prima. **Firmware e nodo vanno aggiornati in coppia.**
- **`yaw_scale` default → 1.0** nel nodo (firmware ora yaw ×1).
- **Nuovi topic pulsanti:** `emergency_stop`, `button_1/2/3` (oltre a `left/right_button`).
- **Alias udev** `/dev/aira_controller` (regola in `udev/99-aira-controller.rules`); il launch
  usa quello come default (override `serial_port:=...`).

**⚠️ BLOCCO ATTUALE — riprendere da qui stasera:**
Dopo il flash, la Blue Pill **non si enumera su USB** sul Pi. `dmesg` mostra tentativi che
falliscono: `device descriptor read/64, error -32`, `Device not responding to setup address`,
`device not accepting address, error -71`, `unable to enumerate USB device`. Firma di
**problema elettrico/segnale USB, non firmware** (il micro *tenta* di connettersi → USB support
era su CDC, ok). Il micro risulta dietro un **hub VIA Labs** (`2109:3431`). Piano di debug
(sez. 7): **(1)** collegare la Blue Pill **direttamente a una porta del Pi** bypassando l'hub;
**(2)** provare altro cavo dati (l'utente pensa non sia il cavo, "ha sempre funzionato");
**(3)** se persiste → difetto classico Blue Pill: **pull-up D+ (PA12) sbagliato** (10k invece
di 1.5k) → saldare 1.5k tra PA12 e 3V3. Nota: prima funzionava (forse altra unità Blue Pill o
senza hub di mezzo).

**Prossimi passi (dopo aver risolto l'USB):** (a) testare assi + tastini `BL/BR` via
`ros2 topic echo`; (b) verificare direzioni (se invertite, flip del singolo segno nel firmware);
(c) **integrazione col robot**: nodo *teleop* che traduce i joystick in comandi + **nodo safety**
per l'EM STOP (quick-stop software: latch → ODrive IDLE → blocca joystick → reset deliberato +
watchdog su heartbeat WiFi). L'EM stop è voluto come **quick-stop software**, non catena HW.

**Backup git:** GitHub privato `git@github.com:gc-stunt97/ROS2-Remote-Controller.git`
(le modifiche di oggi sono su disco Windows, **da committare/pushare**).

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
  Convenzione (come GUI RobotHex): **Y = avanti/indietro, X = destra/sinistra, Z = yaw**.
  Ogni stick ha in cima un **tastino** (`BL`/`BR`).
- **Pulsanti aggiunti (controller 2026-07):** un **EM STOP a fungo** (`EM`) + **3 pulsanti
  general purpose** (`B1/B2/B3`). Tutti **Normally-Open verso massa**, pull-up software +
  debounce nel firmware. Al momento **non collegati a nessuna funzione** (da decidere).
- **Router RUT241 (Teltonika) ora a bordo** nel nuovo case (era roadmap, vedi Alimentazione).
- **STM32 "Blue Pill"** (STM32F103C8): legge i 6 assi analogici + i 6 pulsanti e manda i
  valori al Pi via **seriale USB** (alias stabile **`/dev/aira_controller`**, vedi §4).

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
STM32 (Blue Pill)  --- legge 6 assi + 6 pulsanti (con debounce)
      │  applica deadzone + scaling → riga JSON (chiavi gia' corrette: Y=avanti, X=laterale)
      │  Serial.println @ 57600 baud  (USB, alias /dev/aira_controller, ~50 Hz)
      ▼
joy_node  (nodo ROS2 "joystick_node")
      │  json.loads, PASS-THROUGH assi (niente piu' swap), pubblica
      ├── /left_joystick_data   (geometry_msgs/Point)   assi joystick sinistro
      ├── /right_joystick_data  (geometry_msgs/Point)   assi joystick destro
      ├── /left_button          (std_msgs/Bool)         tastino stick sinistro (BL)
      ├── /right_button         (std_msgs/Bool)         tastino stick destro (BR)
      ├── /emergency_stop       (std_msgs/Bool)         fungo EM STOP (quick-stop sw)
      └── /button_1|2|3         (std_msgs/Bool)         3 pulsanti general purpose
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
  - `serial_port` (default `/dev/aira_controller` — alias udev; override `serial_port:=/dev/ttyACM0`)
  - `baud` (default `57600`)
  - `reconnect_period` (default `2.0` s)
  - **`yaw_scale`** (default **`1.0`**) → il firmware ora normalizza lo yaw ×1. (Col vecchio
    firmware yaw ×2 si passava `yaw_scale:=0.5`.)
  - I pulsanti sono **opzionali** (`.get(...,0)`): il nodo non crasha se il firmware non li invia.
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
  libreria **ArduinoJson**). Dettagli protocollo + tabella pin completa: `firmware/stm32/README.md`.
- **Pinout (2026-07):** assi `LY=PA1, LX=PA0, LZ=PA2` / `RY=PA5, RX=PA3, RZ=PA4`;
  pulsanti `BL=PB10, BR=PB15, EM=PB14, B1=PB11, B2=PB12, B3=PB13` (tutti `INPUT_PULLUP`,
  NO→massa, debounce 30 ms, `1=premuto`); LED vita `PC13`.
- **Contratto JSON:** `{"LY","LZ","LX","RY","RZ","RX","BL","BR","EM","B1","B2","B3"}` @ 57600 baud, ~50 Hz.

### ✅ Come flashare — ST-Link via SWD (FUNZIONA, fatto il 2026-07-14)
Il DFU via **USB nativo su STM32F103 non esiste** (il bootloader di sistema F103 fa DFU solo
su UART/CAN, non USB) → quella strada è un vicolo cieco. **La via buona è l'ST-Link/SWD**
(4 fili: SWDIO, SWCLK, GND, 3V3). Impostazioni **Arduino IDE** usate con successo:
- Board: `Generic STM32F1 series` → Board part number `BluePill F103C8`
- **Upload method: `STM32CubeProgrammer (SWD)`** ← la voce chiave (NON il menu "Programmer")
- Prerequisiti: **STM32CubeProgrammer** installato + driver ST-Link.
- Se non connette (cloni): "Connect Under Reset" in CubeProgrammer + pin NRST, oppure trucco
  BOOT0=1 → reset → upload → BOOT0=0.

> Nota: dopo il flash SWD il **bootloader Maple** (vecchio, `1eaf:0004/0003`) risulta
> sovrascritto; ora la scheda si presenta come **USB CDC STMicroelectronics `0483:5740`**.

Compilazione headless sul Pi (produce il `.bin`, se serve):
```bash
arduino-cli compile -b STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 \
  --output-dir ~/fw_out firmware/stm32/ROS2controller
```

---

## 7. Problemi noti / gotcha (imparati sul campo)

- **DFU via USB su STM32F103 NON esiste** (bootloader di sistema F103 = solo UART/CAN).
  Flashare **sempre via ST-Link/SWD** (vedi §6). Il DFU dal Pi ARM64 non va comunque.
- **⚠️ USB non enumera dopo il flash (BLOCCO ATTUALE 2026-07-14):** `dmesg` mostra
  `error -32`/`error -71`, `Device not responding to setup address`, `unable to enumerate`.
  È **segnale/elettrico**, non firmware. Debug: **(1)** collega la Blue Pill **diretta al Pi**,
  non tramite l'hub VIA Labs `2109:3431`; **(2)** cavo dati alternativo; **(3)** se persiste,
  **pull-up D+ (PA12)** sbagliato sulla Blue Pill (10k→1.5k tra PA12 e 3V3). `lsusb` non mostra
  `0483:5740` finché non enumera.
- **La GUI da SSH** dà `no $DISPLAY`: serve `export DISPLAY=:0` (o lanciarla dal 7", o via
  lo script/icona che lo gestiscono).
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

1. ✅ **Flash firmware via ST-Link** — FATTO (tastini + EM/B1-B3 + yaw ×1). ⏳ **Sbloccare
   l'USB** (enumerazione fallita, vedi §0/§7) per poter leggere la seriale.
2. **Integrazione col robot (il pezzo grosso):** nodo *teleop* che sottoscrive
   `right_joystick_data` (avanti = velocità, yaw = rotazione) e `left_joystick_data`, e
   traduce in **parametri del gait engine** del robot (vedi `ROBOTHEX_HANDBOOK.md`).
3. **Nodo safety / EM STOP:** quick-stop software → latch su `/emergency_stop` → ODrive IDLE →
   blocca joystick → reset deliberato; + **watchdog** su heartbeat (fail-safe se cade il WiFi).
4. **Rete:** far parlare controller e robot via WiFi (stesso `ROS_DOMAIN_ID`, DDS discovery).
5. (Poi) streaming video FPV su pipeline dedicata — vedi handbook robot sez. 6b.

---

## 10. Uso con AIRA (base mobile) — pacchetto `aira_dashboard`

Il controller è **condiviso**: lo stesso hardware (STM32 + joystick) pilota sia RobotHex sia
la base mobile **AIRA** (repo `gc-stunt97/AIRA_Robot`). Cambia solo la GUI che gli metti sopra.

- **`src/aira_dashboard/`** — dashboard dedicata alla base mobile (gemella di `joypad_gui.py`).
  Riusa il nodo joystick `joypad_controller/joypad_node`. Mostra: telemetria ODrive (da
  `/odrive_status`), velocità comandata (`/cmd_vel`) vs reale (`/odom`), mini-mappa odom,
  e un toggle di sicurezza **MOTORI ON/OFF** (chiama `/odrive_node/enable_motors`, con conferma).
  La base si guida con lo **stick SINISTRO** (y=avanti, x=sterzo).
- **`viz/aira/`** — RViz per AIRA sul controller: `aira_base.urdf` (copia piatta generata
  dallo xacro del repo AIRA), `display.launch.py`, `aira_base.rviz` (fixed frame `odom`,
  mostra modello + odometria + `/scan` quando ci sarà il LiDAR).
- **Avvio:** `ros2 launch aira_dashboard aira.launch.py` (joystick + dashboard). Serve
  `python3-tk`. Icona desktop dedicata: `desktop/AIRA-Controller.desktop` +
  `desktop/aira-controller.sh` (installazione come per RobotHex, vedi `desktop/README.md`).
- **Rete:** stesso `ROS_DOMAIN_ID` del robot AIRA (come per RobotHex).
- **Build:** `colcon build --symlink-install --packages-select aira_dashboard` in `~/ros2_ws`.
