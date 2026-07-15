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

## 0. Stato attuale — RIPRENDI DA QUI (2026-07-14, sera)

**Controller ricostruito da zero** (nuovo case stampato 3D con alloggiamento per il router
**RUT241 a bordo**), ricablato completamente, STM32 riflashato.

> ## ✅ IL TELECOMANDO È COMPLETO E VERIFICATO END-TO-END
>
> Assi coerenti (avanti = `Y+`, destra = `X+`), tastini sui pin giusti, yaw a ±1, alias udev
> attivo e stabile **anche dopo un riflash**. Funzionano **entrambe le plance** (RobotHex e
> AIRA), ognuna con la sua icona sul desktop del Pi. Tutto committato e pushato.
>
> ## ▶️ PROSSIMO PASSO: LA RETE — vedi **sez. 11** (piano deciso, da implementare)
>
> Il RUT241 è a bordo e alimentato; manca il cavo Ethernet al Pi e tutta la configurazione.
> La sez. 11 ha il piano completo campo per campo: **non riprogettarlo, eseguilo.**

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
5. **Funzioni dei pulsanti:** `B1/B2/B3` e i tastini stick funzionano ma **non fanno niente** —
   da decidere.
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
| **Rete di casa** | `192.168.1.0/24`, gateway `192.168.1.1` | DHCP |
| **RUT241 (rete privata)** | `192.168.10.1` | LAN spostata a mano dal default |
| **Pi controller — `eth0`** | **`192.168.10.10`** | **statico** (MAC `e4:5f:01:7e:4d:43`) |
| **Pi controller — `wlan0`** | `192.168.1.127` | DHCP di casa → internet |
| **Robot — WiFi** | **`192.168.10.20`** | ⏳ da fare, sez. 11.7 |

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

### 11.7 Robot sulla rete privata (⏳ DA FARE — prossimo passo)

Il robot è **headless**: se si sbaglia la configurazione WiFi diventa muto e si finisce a
smontare la SD. Quindi la regola è: **non si tocca la connessione di casa, se ne AGGIUNGE una
seconda.** NetworkManager tiene più profili e sceglie per priorità: si dà ad `AIRA-LINK`
priorità più alta, e **se il RUT241 è spento il robot torna da solo sulla WiFi di casa** —
rete di sicurezza automatica.

Conseguenze da mettere in conto quando il robot è sulla rete privata:
- **non lo raggiungi più direttamente dal PC Windows** (sta su `192.168.1.x`, il robot su
  `192.168.10.x`) → si passa dal Pi controller con **ProxyJump** in `~/.ssh/config`;
- **niente internet sul robot** (`git pull`/`apt`) finché non si fa il WiFi-as-WAN (11.4).

### 11.8 Da dove ripartire

Fatto: 11.3 (sottorete + controller statico). Poi: **11.7** (robot), poi **11.5** (config
CycloneDDS sulle due macchine), poi **11.4** se si vuole internet sulla rete privata.
