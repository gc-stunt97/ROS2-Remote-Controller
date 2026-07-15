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

## 0. Stato attuale — RIPRENDI DA QUI (2026-07-15)

**Controller ricostruito da zero** (nuovo case stampato 3D con alloggiamento per il router
**RUT241 a bordo**), ricablato completamente, STM32 riflashato.

> ## ✅ IL TELECOMANDO È COMPLETO E VERIFICATO END-TO-END
>
> Assi coerenti (avanti = `Y+`, destra = `X+`), tastini sui pin giusti, yaw a ±1, alias udev
> attivo e stabile **anche dopo un riflash**. Funzionano **entrambe le plance** (RobotHex e
> AIRA), ognuna con la sua icona sul desktop del Pi. Tutto committato e pushato.
>
> ## ✅ MODO MOUSE (15/07): gli stick pilotano il cursore di Ubuntu — sez. **12**
>
> Fuori dalle plance lo **stick destro è il puntatore** (+ scroll con lo yaw, click/drag/tasto
> destro col tastino). Parte da solo all'accensione e **si fa da parte da solo** quando apri una
> plancia. Verificato sul campo: preciso, e il rientro dopo una plancia è ~2 s.
>
> ## ⛔ MAI accettare l'aggiornamento a Ubuntu 24.04 — sez. **13**
>
> **ROS Humble esiste solo su 22.04.** Accettarlo = niente più ROS, plance, joystick, e si
> porterebbe via anche il lavoro sul robot. Il popup è già comparso il 15/07: neutralizzato con
> `Prompt=never`. Gli aggiornamenti **ordinari** (684 in attesa, sistema fermo a luglio 2023) si
> possono fare, ma col Pi davanti e **una copia della SD prima** — mai di fretta, mai via SSH.
>
> ## ▶️ RIPRENDI DA QUI (2026-07-15): LA RETE, sez. **11.7** — ⚠️ **SERVE IL ROBOT ACCESO**
>
> Il link privato RUT241 **è vivo**: router su `192.168.10.1`, Pi controller cablato e statico
> su `192.168.10.10`, AP `ROS2Remote` funzionante (col telefono agganciato si apre la dashboard).
> **Il robot però non tiene la connessione**: si collega, poi ripiega da solo su quella di casa.
> **Primo comando:** `iw wlan0 get power_save` **sul robot** → se è `on` è il sospettato numero
> uno (il Raspberry addormenta la radio di default: spiega il "dopo qualche minuto", ed è gratis).
> Poi `nmcli device wifi list | grep -i ROS2Remote` **dal robot** — la potenza che conta è quella
> vista da lui.
>
> ⚠️ **Antenna nel case, connettore sbagliato e "preferisce casa perché più forte" sono ESCLUSI**
> (15/07, sez. 11.7.1): non ripartire da lì. **E non si diagnostica senza il robot:** chi cade è
> lui, misurare il router da un altro dispositivo dice solo se il router è sano.
>
> Il **test del discovery DDS non è mai stato eseguito**: vedi 11.8 punto 3 — probabilmente
> non serve nessuna configurazione, ma va provato.

**Fatto in questa sessione (2026-07-14, sera):**
- **RISOLTO il blocco USB** che bloccava tutto: la Blue Pill non enumerava (`descriptor read
  error -32`, `error -71`, `unable to enumerate`). **Causa: `USB support` su `None`** nell'IDE
  Arduino → sketch compilato **senza stack USB CDC**. Il pull-up su D+ della Blue Pill è
  cablato in hardware, quindi l'host vedeva il device presentarsi e nessuno rispondeva ai
  descrittori: **un sintomo che urla "guasto elettrico" ed è configurazione pura.**
  Fix: `USB support` = `CDC (generic 'Serial' supersede U(S)ART)` + riflash SWD.
  ⚠️ *Questo stesso file, prima, dava per scontato "USB support era su CDC, ok" — ed era
  proprio quello. Lezione: un'assunzione non verificata scritta nell'handbook fa perdere ore.*
- **Tastini `BL`/`BR` invertiti** rispetto al cablaggio reale → corretti **alla sorgente** nel
  firmware (ora `BL=PB10`, `BR=PB15`), non compensati a valle.
- **Regola udev installata sul Pi**: `/dev/aira_controller` ora esiste davvero (era scritta dal
  14/07 pomeriggio ma non era **mai** stata copiata in `/etc/udev/rules.d/`, quindi non aveva
  mai funzionato nemmeno una volta).
- **Icona AIRA installata** sul desktop del Pi (l'app non partiva perché `aira_dashboard` non
  era mai stato compilato lì: `colcon build` senza `--packages-select`).
- **Due falsi allarmi** da ricordare: (1) gli stick sembravano avere un offset di centro
  (`LY=-0.12`) → era **il telecomando appoggiato sul letto** che spingeva gli stick;
  (2) X/Y invertiti e Z fermo a 0.50 → **non erano due bug ma uno solo**, il `joy_node`
  vecchio ancora in esecuzione / non pullato sul Pi. Vedi sez. 7.

**Fatto in questa sessione (2026-07-14, pomeriggio):**
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

**Prossimi passi, in ordine:**
1. **Rete: link privato RUT241** → **sez. 11**. È il prerequisito di tutto il resto: finché
   robot e controller stanno sulla WiFi di casa, il link (e quindi l'EM stop software)
   dipende da un router che non è a bordo.
2. **Integrazione col robot**: nodo *teleop* che traduce i joystick in comandi.
3. **Nodo safety / EM STOP**: quick-stop software (latch su `/emergency_stop` → ODrive IDLE →
   blocca joystick → reset deliberato) + **watchdog su heartbeat**, fail-safe se cade il link.
   L'EM stop è voluto come **quick-stop software**, non catena di sicurezza HW.
4. Decidere cosa fanno `B1/B2/B3` e i tastini degli stick: oggi **funzionano ma non sono
   collegati a nessuna funzione**.

**Backup git:** GitHub privato `git@github.com:gc-stunt97/ROS2-Remote-Controller.git`.
Tutto il lavoro del 14/07 è **committato e pushato** su `main` (`f3b2c5e`, `fa289dd`,
`bbf4682`). Il deploy è: si pusha su `main` da Windows, il Pi fa `git pull` + `colcon build`.

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
   oppure
cursor_node (modo mouse, sez. 12) → mouse virtuale uinput → cursore di Ubuntu
```

⚠️ **`joy_node` è l'unico che apre la seriale, e ce ne può essere UNO SOLO ACCESO** (plancia
RobotHex, plancia AIRA, o modo mouse). Non è una convenzione: due lettori si rubano i byte e
non funziona nessuno dei due, **senza dare errori**. Vedi sez. **12.3**.

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
- **`cursor_node.py`** → **modo mouse** (sez. **12**): sottoscrive `right_joystick_data` /
  `right_button` / `button_1` e li traduce in un **mouse virtuale uinput**. Parametri:
  `stick` (`right`), `speed` (900 px/s), `deadzone` (0.15), `expo` (2.0), `rate` (90 Hz),
  `invert_y`, `scroll_deadzone`/`scroll_speed`/`invert_scroll`, `click_freeze`, `data_timeout`,
  **`long_press_time`** (0.5 s; `0` = click diretto e niente tasto destro dal tastino),
  `right_click_topic` (`button_1`; `""` = disattivato).
- **`mouse.launch.py`** → `joy_node` (col nome **`joystick_node_mouse`**, vedi 12.5) +
  `cursor_node`. Se muore il cursore cade tutto, per non tenere la seriale a vuoto.
- Eseguibili ROS2: **`joypad_node`**, **`joypad_gui_app`**, **`cursor_node`**.
- Dipendenze di sistema: `python3-serial`, `python3-tk`, **`python3-evdev`** (modo mouse).

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
- **⚠️⚠️ USB non enumera dopo il flash → CONTROLLA IL MENU PRIMA DI DISSALDARE.**
  Sintomo: `dmesg` dà `descriptor read error -32`, `error -71`, `unable to enumerate`, e
  `lsusb` non mostra `0483:5740`. **Sembra un guasto elettrico. Quasi sempre non lo è.**
  **Causa reale (2026-07-14, costata mezza serata): `Tools → USB support` su `None`** nell'IDE
  Arduino → lo sketch gira benissimo (LED compreso) ma **non ha stack USB CDC**, e `Serial`
  finisce sulla UART1 (PA9/PA10). Il pull-up su D+ della Blue Pill è **cablato in hardware**,
  quindi l'host vede il device presentarsi e poi non risponde nessuno: **da qui la firma
  "elettrica" ingannevole.** Fix: `USB support` = `CDC (generic 'Serial' supersede U(S)ART)`
  e riflash.
  Controlli in ordine, dal gratis al fastidioso: **(1)** il menu `USB support`; **(2)** jumper
  **BOOT0 = 0** (a 1 gira il bootloader di sistema, che su F103 è solo UART → stesso sintomo);
  **(3)** LED PC13: il firmware lampeggia a **1 Hz** (500 ms) — se lampeggia ~9× più lento il
  quarzo da 8 MHz non oscilla e l'USB non può funzionare (servono 48 MHz esatti); se non
  lampeggia, il firmware non gira. **Solo se tutto questo è a posto** passare all'elettrico:
  Blue Pill diretta al Pi senza l'hub VIA Labs `2109:3431`, altro cavo dati, e come ultima
  ipotesi il pull-up D+ (PA12) sbagliato (10k→1.5k tra PA12 e 3V3).
- **⚠️ X/Y invertiti nella GUI *e* Z fermo a 0.50 = UN SOLO problema, non due:** sul Pi gira il
  **`joy_node` vecchio** (quello che compensava lo swap e dimezzava lo yaw) mentre il firmware
  è già nuovo. Fix: `git pull` + rebuild + **riavviare il nodo**. **Non toccare il codice**:
  `joy_node` e le due plance sono già pass-through corretti.
- **Il `git pull` non sostituisce un nodo già in esecuzione.** Chiudere e riaprire la GUI basta
  (il launch spegne tutto quando chiudi la finestra, event handler in fondo ai launch file).
  Un nodo partito ore prima gira ancora col codice di allora: sintomo classico "ho aggiornato
  ma non cambia niente".
- **Con USB CDC il baud di `Serial.begin(57600)` è virtuale e ignorato**: un mismatch di baud
  lato `joy_node.py` **non può** essere la causa di un problema.
- **Prima di dare la colpa al firmware, guarda il telecomando.** Il 14/07 gli assi Y uscivano
  a `-0.12`/`-0.19` da fermi: era il telecomando **appoggiato sul letto** che spingeva gli
  stick. Il firmware stava dicendo la verità.
- **Un'app che non parte dall'icona muore muta** (`Terminal=false`): lanciare lo script a mano
  (`~/aira-controller.sh`) per vedere l'errore. Causa tipica: pacchetto mai compilato su quel
  Pi, perché si è fatto `colcon build --packages-select <altro>`.
- **La GUI da SSH** dà `no $DISPLAY`: serve `export DISPLAY=:0` (o lanciarla dal 7", o via
  lo script/icona che lo gestiscono).
- **USB del Pi si "impunta"** dopo tanti reset/flash falliti (la scheda sparisce da
  `lsusb`): **NON è rotta** — si risolve con `sudo reboot` del Pi.
- **Blue Pill:** connettore micro-USB fragile, maneggiare i cavi con delicatezza.
- **`unattended-upgrades`** (auto-updater di Ubuntu) può riempire `/boot/firmware` (piccola)
  e bloccare `apt`: se ricapita, liberare i `.bak` in `/boot/firmware` e riavviare. Si può
  disattivare l'updater per trattare il controller come appliance.
- **⚠️ Gli stick sembrano morti, in TUTTE le app, senza un solo errore** → quasi certamente
  **due processi stanno leggendo la stessa seriale** e si rubano i byte (Linux non lo
  impedisce). Vedi sez. **12.3**. Controllo: `~/mouse-mode.sh status` dice chi tiene
  `/dev/aira_controller`; deve essere **uno solo**. Causa tipica: due plance insieme, oppure il
  testimone del modo mouse che non ha morso.
- **"Funziona quando lo lanci da SSH, non quando parte dall'icona"** → **è un gruppo Unix**: la
  sessione grafica è più vecchia di un `usermod -aG` e i gruppi si leggono **al login**. Serve
  logout/reboot. Vedi sez. 12.5 punto 2.
- **⛔ Il popup "è disponibile Ubuntu 24.04": NON accettarlo mai.** Distruggerebbe ROS Humble.
  Vedi sez. **13** (già neutralizzato con `Prompt=never` il 15/07).

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

1. ✅ **Telecomando completo e verificato** — FATTO il 14/07: flash via ST-Link, USB sbloccato,
   assi/tastini/yaw corretti, alias udev attivo, entrambe le plance funzionanti. Vedi §0.
2. ⏳ **Rete: link privato RUT241** — piano deciso e scritto, **da eseguire**: vedi **sez. 11**.
   È il prerequisito dei punti 3 e 4: l'EM stop software vale quanto vale il link su cui viaggia.
3. **Integrazione col robot (il pezzo grosso):** nodo *teleop* che sottoscrive
   `right_joystick_data` (avanti = velocità, yaw = rotazione) e `left_joystick_data`, e
   traduce in **parametri del gait engine** del robot (vedi `ROBOTHEX_HANDBOOK.md`).
4. **Nodo safety / EM STOP:** quick-stop software → latch su `/emergency_stop` → ODrive IDLE →
   blocca joystick → reset deliberato; + **watchdog** su heartbeat (fail-safe se cade il link).
5. **Funzioni dei pulsanti:** `B2/B3` e il tastino **sinistro** funzionano ma **non fanno
   niente** — da decidere. ✅ Nel **modo mouse** (sez. 12) il tastino **destro** fa
   click/drag/tasto destro e `B1` fa il tasto destro — ma **solo lì**: dentro le plance sono
   ancora liberi. Il **fungo** resta da collegare al nodo safety (punto 4).
6. (Poi) streaming video FPV su pipeline dedicata — vedi handbook robot sez. 6b.
   ⚠️ Attenzione all'interazione con la sez. 11.4 (radio singola del RUT241).

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
- **Rete:** stesso `ROS_DOMAIN_ID` del robot AIRA (come per RobotHex). Vedi **sez. 11**.
- **Build:** `colcon build --symlink-install --packages-select aira_dashboard` in `~/ros2_ws`.

---

## 11. Rete — link privato RUT241 (PIANO DECISO, da implementare)

> **STATO (2026-07-15): lato CONTROLLER FATTO E VERIFICATO.** Manca il robot (sez. 11.7) e
> l'internet opzionale (sez. 11.4). **SIM/LTE: non la si usa**, non interessa.

### 11.0 Valori reali (la mappa della rete, aggiornarla se cambia)

| Cosa | Indirizzo | Come |
|---|---|---|
| **Rete di casa** | `192.168.1.0/24`, gateway `192.168.1.1` | DHCP. SSID `Linkem_84AB80` |
| **RUT241 (rete privata)** | `192.168.10.1` | LAN spostata a mano dal default |
| **WiFi privata** | SSID **`ROS2Remote`** | password: **non in repo**, chiedere a Giulio |
| **Pi controller — `eth0`** | **`192.168.10.10`** | **statico** (MAC `e4:5f:01:7e:4d:43`) |
| **Pi controller — `wlan0`** | `192.168.1.127` | DHCP di casa → internet |
| **Robot AIRA — WiFi privata** | **`192.168.10.20`** | statico, profilo `ROS2Remote` ⚠️ instabile, 11.7 |
| **Robot AIRA — WiFi di casa** | `192.168.1.234` | DHCP, profilo `Linkem_84AB80` (fallback) |
| **RobotHex** | `192.168.10.21` | riservato, non ancora fatto |

`ROS_DOMAIN_ID` sul controller = **vuoto (= 0)**. Da verificare che il robot dica lo stesso.

**Accesso da Windows** (`~/.ssh/config`, già configurato): `controller` → `192.168.1.127`;
`robot` → `192.168.10.20` **via ProxyJump dal controller**; `robot-casa` → `192.168.1.234`
(quando il robot ripiega sulla WiFi di casa).
⚠️ **Non usare i nomi `.local` da questo PC Windows:** `Controller.local` e `Robot.local`
risolvono entrambi a **`127.0.0.2`** (VS Code finisce su sé stesso e la connessione fallisce).
Qualcosa intercetta l'mDNS — sospetto l'OpenVPN installato. Usare gli host del config SSH.

⚠️ **Il RUT241 di fabbrica sta su `192.168.1.1` — lo stesso indirizzo del router di casa.**
È la prima cosa da cambiare, prima di collegare qualsiasi cosa: il 14/07 il cavo è stato
attaccato con le sottoreti ancora coincidenti e il risultato è stato due DHCP che
distribuivano indirizzi in conflitto, mDNS impazzito e `Controller.local` che risolveva a
`127.0.0.2` (SSH da VS Code morto). Per configurare il router **serve un dispositivo che
stia solo sulla sua rete** (il telefono, con i dati mobili spenti): da un PC collegato a
casa, `192.168.1.1` ti porta sul router di casa, non sul RUT241.

**Il Pi del controller è bi-residente ed è il ponte:** sta su casa via `wlan0` e sulla rete
privata via `eth0`. Da Windows si raggiunge il robot passando da lui (ProxyJump).

### 11.1 Perché (il problema che risolve)

Oggi robot e controller stanno **entrambi sulla WiFi di casa**: funziona, ma il link dipende
da un router che **non è a bordo**. Fuori portata, a casa d'altri, o se cade la corrente al
router di casa → il telecomando smette di comandare. E siccome l'**EM stop è software** (il
fungo funziona solo se il messaggio arriva), la rete **non è un comfort: è parte della catena
di sicurezza**.

Col RUT241 a bordo **la rete la porti con te**: router e telecomando sono lo stesso oggetto,
alimentato dalla stessa powerbank. Se c'è il telecomando, c'è la rete.

### 11.2 Topologia

```
   [Pi controller] --cavo Ethernet--> [porta LAN del RUT241]
                                             |
                                        (WiFi privata "AIRA-LINK")
                                             |
                                        [Pi robot]
```

- **Il Pi del controller va nella porta LAN. MAI la WAN.** Per un router, **WAN = la porta che
  guarda "fuori"** (verso internet) e **LAN = le porte per la roba tua**; tra le due c'è un
  firewall. Mettendo il Pi in WAN diresti al router "questo coso è internet" → il router lo
  firewalla e **il robot in WiFi non riesce a parlargli**. Sembra un guasto, è il router che fa
  il suo mestiere.
- Pi e router sono a 10 cm dentro lo stesso case: **cablato**, sarebbe assurdo farli parlare via radio.
- Il traffico robot↔controller **non esce mai dal RUT241** (entra dalla radio, esce dal cavo):
  non passa da casa nemmeno se la WiFi di casa c'è.

### 11.3 Configurazione — FATTA lato controller (2026-07-15)

1. ✅ **LAN del RUT241 spostata** da `192.168.1.1` a **`192.168.10.1`** (Network → Interfaces →
   LAN → Edit). Applicando, il dispositivo da cui stai configurando **si scollega**: è il
   risultato atteso, non un errore. Poi l'interfaccia vive su `http://192.168.10.1`.
2. ✅ **Pi controller → porta LAN** del RUT241, con **IP statico lato Pi**, non riservazione DHCP:

   ```bash
   sudo nmcli connection modify "Wired connection 1" \
     ipv4.method manual ipv4.addresses 192.168.10.10/24 \
     ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
   sudo nmcli connection up "Wired connection 1"
   ```

   **Perché statico e non riservazione DHCP:** la riservazione dipende dal router (resetti o
   sostituisci il RUT241 → salta). L'IP statico vive sulla macchina e sopravvive a tutto.
   `.10`/`.20` sono **fuori dal pool DHCP** (che parte da `.100`), quindi non collidono.
   **`gateway ""` + `dns ""` + `never-default` sono la parte importante:** il cavo è un
   *collegamento*, non un'*uscita*. Vedi il gotcha qui sotto.
3. ⏳ **Robot** → `192.168.10.20`: sez. 11.7.

> ### ⚠️ Gotcha: due gateway, e il Pi sceglie quello sbagliato
> NetworkManager assegna metrica **100 all'Ethernet** e **600 al WiFi** (più basso = vince),
> presumendo che il cavo sia meglio della radio. Qui è **falso**: il cavo porta al RUT241, che
> non ha nessuna uscita. Appena collegato, il Pi ha smesso di avere internet — e il sintomo è
> subdolo perché **il DNS continuava a funzionare** (`getent hosts github.com` risolveva) mentre
> i pacchetti sparivano. La prova regina è `ping 8.8.8.8` → `From 192.168.10.1 Destination Net
> Unreachable`: **è il router stesso che dichiara di non sapere dove mandarli.**
> Fix già applicato e persistente: `ipv4.never-default yes` sulla connessione via cavo.

### 11.4 Internet (opzionale, si può)

Il RUT241 può collegarsi **lui** alla WiFi di casa come se fosse un telefono e ridistribuire
quell'internet alla rete privata: in RutOS si chiama **WiFi as WAN** (la porta WAN fisica resta
vuota, il "fuori" arriva via radio). Così robot e controller si parlano in privato **e** hanno
internet per `git pull` / `apt`. Se la WiFi di casa cade, **il teleop continua a funzionare**:
perde solo internet.

⚠️ Prezzo: il RUT241 ha **una sola radio 2.4 GHz**, che così fa da AP e da client sullo stesso
canale. Per i joystick (decine di KB/s) irrilevante. **Quando arriverà il video FPV** (roadmap
sez. 9.5) potrebbe diventare stretto → valutare se spegnere il WiFi-as-WAN mentre si vola.

### 11.5 ⚠️ Il discovery DDS (la trappola da prevedere PRIMA)

Quando un nodo ROS2 parte **non sa chi altro c'è**: per scoprirlo **urla nella stanza**, cioè
manda un messaggio indirizzato a "chiunque sia in ascolto" (**multicast**) che dice "ciao, sono
`joy_node`, pubblico `left_joystick_data`". Chi è interessato risponde e da lì in poi si parlano
diretti. Questa fase si chiama **discovery**.

**Un access point WiFi spesso si rifiuta di ripetere quelle urla tra i suoi client.** In un bar
è giusto (non vuoi che il telefono di uno sconosciuto veda il tuo), per noi è un disastro. Due
impostazioni lo fanno: la **client isolation** e la gestione **multicast / IGMP snooping**.

Sintomo velenoso: i topic ci sono su entrambi i lati **ma non arriva niente**, `ros2 node list`
ne vede metà. **Non sembra un problema di rete: sembra codice rotto.**

**Soluzione scelta: togliere di mezzo le urla.** Con due sole macchine si usa **CycloneDDS con i
peer unicast espliciti**: si dice a ciascuna l'IP dell'altra e il multicast non serve più.
Deterministico, e indipendente da come è configurato l'AP. Prerequisito: gli **IP fissi** di 11.3.
(Più il solito `ROS_DOMAIN_ID` uguale sulle due macchine.)

### 11.7 Robot sulla rete privata — ⚠️ FATTO MA INSTABILE (blocco attuale)

Il profilo è configurato e **ha funzionato** (SSH dal controller a `192.168.10.20` riuscito),
ma **il robot non tiene la connessione**: dopo qualche minuto è ripiegato da solo sulla WiFi
di casa. **Questo è il punto da cui ripartire.**

Configurazione già applicata sul robot (non serve rifarla):

```bash
sudo nmcli connection add \
  type wifi con-name ROS2Remote ifname wlan0 ssid ROS2Remote \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<password>' \
  ipv4.method manual ipv4.addresses 192.168.10.20/24 \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes \
  connection.autoconnect yes connection.autoconnect-priority 10
```

**✅ La rete di sicurezza funziona, verificato sul campo.** Il robot è **headless**: sbagliare
la sua WiFi significa perderlo. Per questo si è **AGGIUNTO** il profilo senza toccare quello di
casa (`Linkem_84AB80`, priorità 0 contro 10). Quando `ROS2Remote` è caduta, NetworkManager è
tornato da solo a casa e il robot è ricomparso a `192.168.1.234`. **Regola di risalita: spegni
il RUT241 e il robot torna a casa.** Non togliere mai il profilo `Linkem_84AB80`.

Note: `sudo nmcli connection up ROS2Remote` **uccide la sessione SSH** da cui lo lanci (il robot
cambia rete sotto i piedi) → lanciarlo con `sudo nohup ... &` così sopravvive alla morte di SSH.
La password ha un `!`: **apici singoli obbligatori**, tra virgolette doppie bash fa l'espansione
della cronologia e la storpia.

**⏳ DA FARE — diagnosi della caduta.** Il router NON è il problema: risponde al ping dal
controller sul cavo, quindi powerbank e boost DC-DC sono a posto. È la **radio**. Comandi:

```bash
nmcli device wifi list | grep -i ROS2Remote        # <- POTENZA DEL SEGNALE: e' il numero chiave
journalctl -u NetworkManager --since "20 min ago" | grep -i -E 'ROS2Remote|wlan0|disconnect|assoc' | tail -25
```

Sotto il 40-50% di segnale siamo in zona rossa e nessuna configurazione software salva niente.

#### 11.7.1 Ipotesi ESCLUSE (2026-07-15, sessione senza robot — non ripercorrerle)

- ❌ **"L'antenna del RUT241 è soffocata nel case."** Era l'ipotesi principale: **è FALSA.**
  Il pannellino frontale del router è stato rimosso e il RUT241 è avvitato **al lato del case**,
  con i **connettori originali che sporgono fuori**. Niente prolunghe coassiali (quindi niente
  perdite di cavo, niente connettori aggiunti). **L'antenna è in aria libera, fuori dal case.**
- ❌ **"L'antenna è avvitata sul connettore sbagliato (WiFi nudo)."** Non è possibile confonderli:
  sul RUT241 la porta **WiFi è l'unica col pin maschio** ("pirulino") e le due **LTE hanno il foro**
  (RP-SMA vs SMA: invertono il genere del pin, non la filettatura). L'antenna WiFi ha il foro →
  **fa contatto solo sulla porta WiFi**. Verificato a vista. Le antenne LTE non sono montate.
- ❌ **"Il robot preferisce la WiFi di casa perché è più forte."** **NetworkManager non fa
  'l'erba del vicino':** da connesso non rivaluta le alternative. `autoconnect-priority` pesa
  **solo nel momento della scelta**, cioè quando non è attaccato a niente. → **Il fallback su
  `Linkem_84AB80` non è la causa, è la conseguenza:** il robot è tornato a casa *perché
  ROS2Remote gli è caduta*. **La domanda giusta è "perché cade la privata", non "perché sceglie
  casa".**
- ❌ **"Il RUT241 trasmette a potenza ridotta."** **L'AP è sano:** da PC **associato** a 1 m il
  segnale è **99%**, fondo scala, con DHCP, ping e internet a posto in contemporanea.
  ⚠️ **Lezione sulla misura:** lo stesso AP, misurato in **scansione passiva** (PC non
  associato), dava **90%** e sembrava fiacco — perché scansionando la scheda saltella tra i
  canali e prende un beacon al volo. **La scansione passiva è pessimista: non usarla per
  giudicare un AP.** Il numero buono si legge **da associati**
  (`netsh wlan show interfaces`, o `nmcli device wifi list` sul lato Linux mentre è connesso).

#### 11.7.2 Piste vive, in ordine (per la prossima sessione COL ROBOT)

1. ⭐ **Power save del WiFi sul Pi del robot.** Il Raspberry ha il risparmio energetico della
   radio **attivo di default**: quando il traffico cala la radio si addormenta, e NetworkManager
   a un certo punto conclude che l'AP non risponde più → molla e ripiega su casa. **Spiega il
   "dopo qualche minuto" meglio di qualsiasi ipotesi hardware, ed è gratis.**
   Check: `iw wlan0 get power_save` → se `on`, è un forte sospettato.
   Fix: `sudo iw wlan0 set power_save off` (per renderlo persistente: `wifi.powersave = 2` in
   un file sotto `/etc/NetworkManager/conf.d/`).
2. **Il log del robot al momento della caduta** — vedi i comandi sopra. È l'unica cosa che dice
   *cosa* è successo invece di *cosa potrebbe*. Con `journalctl` si vede se ha perso
   l'associazione (radio) o se è NetworkManager ad aver deciso.
3. **Portata reale a distanza** — l'AP è sano a 1 m (99%), ma la caduta avviene *in movimento*.
   Da misurare **dal robot, dove sta il robot**, non dal tavolo.

**Dati dell'AP (2026-07-15):** SSID `ROS2Remote`, canale **4**, WPA2-Personal/CCMP, BSSID
`20:97:27:03:f2:a9`, 2.4 GHz. Pool DHCP dal `.100` (il PC ha preso `.158`).
🔑 **La password WiFi è quella ORIGINALE di fabbrica, stampata sull'etichetta del router**
(≠ password di admin — è l'inghippo che il 15/07 ha fatto perdere tempo: il PC non si connetteva
perché si provava quella sbagliata). Non è nel repo: sta sul RUT241.

> **Metodo, imparato a caro prezzo il 15/07:** chi cade è **il robot**, quindi le misure che
> contano si fanno **dal robot**. Caratterizzare il router da un altro dispositivo dice se il
> router è sano, **non** perché il robot molla. Senza il robot davanti, questa diagnosi non si
> chiude: non inseguire ipotesi a distanza.

### 11.8 Da dove ripartire

1. ✅ 11.3 — sottorete del router + controller statico sul cavo. **Fatto e verificato.**
2. ⚠️ **11.7 — il robot non tiene la connessione. PARTI DA QUI, E SERVE IL ROBOT ACCESO.**
   Ordine: (a) `iw wlan0 get power_save` sul robot — vedi 11.7.2 punto 1, è il candidato
   migliore e si spegne in un comando; (b) `nmcli device wifi list | grep -i ROS2Remote`
   **dal robot** — la potenza vista da lui è il numero che conta; (c) `journalctl` al momento
   della caduta. **Le ipotesi di 11.7.1 sono già state escluse: non ripartire da quelle.**
3. ⏳ **Test discovery DDS — mai eseguito**, il robot è caduto prima. Vedi 11.5: **provare
   PRIMA se serve davvero.** La trappola del multicast riguarda i client WiFi *tra loro*, ma
   qui il robot è in WiFi e il controller sul cavo, e il bridge del router di solito passa.
   Test: plancia AIRA sul controller, e sul robot `ros2 topic list` → si vedono
   `left/right_joystick_data`? Se sì **non serve CycloneDDS** e 11.5 si riduce a "non servito".
4. ⏳ 11.4 — WiFi-as-WAN, se si vuole internet sulla rete privata. **Nota:** serve anche per
   installare pacchetti sul robot (oggi sulla rete privata non ha internet: niente `apt`).

---

## 12. Modo mouse — gli stick pilotano il cursore di Ubuntu (FATTO 2026-07-15)

> **✅ FUNZIONA, verificato sul campo:** cursore preciso e fine negli aggiustamenti, click,
> trascinamento, tasto destro, e passaggio automatico da/verso le plance. Parte da solo
> all'accensione.

### 12.1 Cos'è e perché

Fuori dalle plance il telecomando è un PC Ubuntu con un touchscreen: il dito va bene per
premere un toggle, molto meno per il desktop o per **RViz**. Col modo mouse lo **stick destro
diventa il puntatore**, e le mani non lasciano mai il telecomando.

**Chi fa cosa** (default):

| Comando | Azione |
|---|---|
| Stick destro X/Y | muove il cursore |
| Stick destro Z (yaw) | rotella / scroll |
| Tastino stick — tocca e rilascia | click **sinistro** |
| Tastino stick — tieni e **muovi** | **trascinamento** |
| Tastino stick — tieni **fermo** 0.5 s | click **DESTRO** (come il long press del touch) |
| `B1` | click destro diretto |

⚠️ Col long press attivo il click sinistro parte **al rilascio**, non alla pressione: è l'unico
modo per distinguere i tre casi, ed è come si comporta qualsiasi touchscreen. Con
`long_press_time:=0.0` si torna al click immediato (e si perde il tasto destro dal tastino).

### 12.2 Com'è fatto

`joypad_controller/cursor_node.py` sottoscrive i topic che **già esistono**
(`right_joystick_data`, `right_button`, `button_1`) e li traduce in un **mouse virtuale creato
via uinput**.

**Perché uinput e non simulare i click dentro la GUI:** il dispositivo lo crea il **kernel**,
quindi lo vedono *tutte* le applicazioni (desktop, RViz, browser, dashboard) **senza toccare il
codice di nessuna**, e convive col touch (per il sistema sono due dispositivi che muovono lo
stesso cursore). È anche il motivo per cui è **immune ai cambi di X11/Wayland**: non è roba
grafica, è kernel.

Avvio: `mouse.launch.py` = `joypad_node` + `cursor_node`. Gestione:
**`~/mouse-mode.sh {start|stop|status}`**. Icona `Modo Mouse` + **autostart**
(`~/.config/autostart/`).

Dettagli non ovvi, documentati nel nodo: accumulo del resto frazionario (il movimento uinput è
intero: a bassa velocità `int(0.4)` = 0 e il cursore non si muoverebbe **mai**); integrazione a
rate fisso invece che a ogni messaggio; watchdog che ferma il cursore e rilascia i tasti se la
seriale tace (senza, il cursore scapperebbe con l'ultimo valore ricevuto); Y invertito (sullo
schermo Y cresce in basso, sullo stick avanti è Y+).

### 12.3 ⚠️ La seriale vuole UN SOLO lettore (il cuore del problema)

**Linux non impedisce a due processi di aprire la stessa seriale.** Nessun errore, nessuno
viene respinto: **si rubano i byte a vicenda**, le righe JSON arrivano spezzate a entrambi,
entrambi le scartano (giustamente) e **non funziona né il mouse né la plancia** — *senza un solo
messaggio di errore*. Sembra che gli stick siano morti. È lo stesso motivo per cui non si
lanciano due plance insieme.

**Soluzione — passaggio di testimone, automatico:** gli script delle plance chiamano
`mouse-mode.sh stop` prima di partire e `start` alla chiusura. Non c'è niente da ricordare.

Il testimone è scritto così: `[ -x "$HOME/mouse-mode.sh" ] && "$HOME/mouse-mode.sh" stop` →
**se cancelli `~/mouse-mode.sh`, le plance funzionano identiche a prima.** Il modo mouse è
**additivo e reversibile**: per disinstallarlo, `rm ~/.config/autostart/AIRA-Mouse.desktop`.

### 12.4 Installazione (una tantum) — vedi `desktop/README.md`

```bash
sudo apt install -y python3-evdev
sudo cp udev/99-aira-uinput.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=misc   # NON basta "udevadm trigger"
sudo usermod -aG input $USER                                # poi RILOGGATI
cp desktop/mouse-mode.sh ~/ && chmod +x ~/mouse-mode.sh
cp desktop/AIRA-Mouse.desktop ~/Scrivania/
mkdir -p ~/.config/autostart && cp desktop/AIRA-Mouse.desktop ~/.config/autostart/
```

### 12.5 ⚠️ Gotcha imparati il 15/07 (ognuno è costato un giro)

1. **`udevadm trigger` da solo NON applica i permessi.** Emette eventi `change`, e udev applica
   MODE/GROUP **solo sugli `add`**. La regola viene letta, matcha... e `/dev/uinput` resta
   `root:root` come se non esistesse. Serve `--action=add` (o un reboot).
2. **I gruppi si leggono AL LOGIN.** Dopo `usermod -aG input`, la **sessione grafica già aperta
   non ha il gruppo**: il `cursor_node` lanciato dall'icona muore con
   `"/dev/uinput" cannot be opened for writing`, mentre da una sessione SSH *nuova* funziona
   benissimo. **Sintomo micidiale: "funziona quando lo lanci tu, non quando lo lancio io".**
   Serve logout/reboot — nessun comando aggiorna una sessione già aperta.
3. **Uccidere un `ros2 launch` NON porta giù i suoi nodi:** restano orfani e **tengono la
   seriale**. Vanno ammazzati per nome. Per questo il joystick del modo mouse si chiama
   **`joystick_node_mouse`**: con lo stesso nome delle plance le righe di comando sarebbero
   identiche e non si potrebbe fermare l'uno senza ammazzare l'altro. (I topic non cambiano:
   sono assoluti, non dipendono dal nome del nodo.)
4. **SIGTERM non basta:** `joypad_gui_app` (Tk + rclpy) lo ignora e resta appeso *con la seriale
   in mano* — visti zombie sopravvissuti a due SIGTERM di fila. Serve il fallback a `-9`.
5. **`pkill -f` matcha la PROPRIA riga di comando** e si suicida a metà lavoro. Tutti i pattern
   usano il trucco `[x]` (`lib/[j]oypad_controller/`). Successo due volte, la seconda con `-9`.
6. **Gli script `setup.bash` di ROS non sopravvivono a `set -u`** (leggono
   `AMENT_TRACE_SETUP_FILES` non definita): lo script muore sul `source` senza lanciare niente.
   Serve `set +u` attorno.
7. **⚡ Non scansionare `/proc` a mano per sapere chi tiene un device: usa `fuser`.** La
   scansione in bash (244 processi × tutti i loro fd) costa **6,2 s** su questo Pi; `fuser` fa
   lo stesso in **0,13 s**, stesso identico risultato, ed era già installato. Era la causa degli
   ~11 s che il modo mouse ci metteva a tornare dopo una plancia: ora ~2 s (di cui 1,8 è il
   `source` di ROS, incomprimibile).

### 12.6 Robustezza — cosa regge e cosa no

**Regge:** `uinput` è kernel (immune a X11/Wayland e agli upgrade del desktop), la regola udev
sta in `/etc/udev/rules.d/` (roba nostra, gli aggiornamenti non la toccano), `python3-evdev` è
un pacchetto Debian con API ferma. Nessun conflitto col touch o con un mouse USB vero: per il
sistema sono dispositivi diversi che muovono lo stesso cursore.

**Non regge (la colla):** il testimone trova i processi **per nome**. Se si rinomina un
pacchetto, un eseguibile o un nodo, i `pkill` **smettono di trovare qualcosa in silenzio** →
torna la contesa della seriale, col sintomo "gli stick sembrano morti" e nessun errore, magari
settimane dopo il rename. Se un giorno serve irrobustire: PID file o servizio systemd al posto
dei pattern.

**⚠️ Trappola strutturale:** gli script in `~` sono **copie**, non symlink, di `desktop/*.sh`.
Il 15/07 `~/robothex-controller.sh` era una versione **più vecchia** del repo, divergente da
chissà quando. **Ogni volta che si tocca `desktop/*.sh` bisogna ricopiarli sul Pi**, altrimenti
il repo dice una cosa e il Pi ne fa un'altra.

---

## 13. ⛔ MAI aggiornare Ubuntu alla release successiva

**Il Pi ha Ubuntu 22.04 + ROS Humble. ROS Humble esiste SOLO su 22.04.** Non è una preferenza:
ogni versione di ROS è legata a una versione di Ubuntu (su 24.04 c'è ROS *Jazzy*, altra cosa).

Accettare l'offerta di `24.04 LTS` **lascia orfani tutti i pacchetti `ros-humble-*`**: niente
`colcon build`, niente plance, niente joystick, niente modo mouse — e il robot AIRA parla
Humble, quindi si porterebbe dietro anche quel lavoro. Sarebbero giorni per ricostruire.

**Fatto il 15/07** (protezione contro un click distratto sul touch — il popup era già comparso):

```bash
sudo sed -i 's/^Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
# verifica: deve dire  Prompt=never
```

Gli aggiornamenti di sicurezza continuano ad arrivare; sparisce solo il salto di versione.

**Aggiornamenti ordinari (`apt upgrade`):** al 15/07 ce ne sono **684** in attesa e il sistema è
fermo a **luglio 2023** (kernel `linux-image-raspi` compreso, più 3 anni di patch di Humble).
Non sono pericolosi come il salto di release — restano dentro 22.04/Humble — ma **non si fanno
via SSH né di fretta**: Pi davanti, tempo, e **copia dell'immagine della SD prima** (è l'unico
vero "annulla" che esiste su una scheda SD). Mai prima di una sessione in cui serve che il
robot funzioni.
