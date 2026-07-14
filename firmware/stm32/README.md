# Firmware STM32 — controller joystick

Firmware per la **STM32 "Blue Pill"** del controller remoto (replica del design di
James Bruton). Legge i due joystick arcade a 3 assi + i pulsanti e manda i valori
via **seriale USB** al Raspberry del controller, dove il nodo ROS2 `joypad_controller`
(`joy_node.py`) li ripubblica sui topic `left_joystick_data` / `right_joystick_data`.

## Toolchain
- Framework: **Arduino core (STM32duino)**, non HAL/CubeIDE — scelta pragmatica: il
  compito è leggero (6 ADC + seriale a 50 Hz), Arduino è più che sufficiente e manutenibile.
- Libreria: **ArduinoJson**.
- File: `ROS2controller.ino`.

## Pin (Blue Pill)
Convenzione (come GUI RobotHex): **Y = avanti/indietro, X = destra/sinistra**.
Le etichette firmware sono ora coerenti col cablaggio fisico.

| Segnale | Pin  | Note |
|---------|------|------|
| LY / LX / LZ (joystick sinistro) | PA1 / PA0 / PA2 | ADC — Y=avanti, X=laterale, Z=yaw |
| RY / RX / RZ (joystick destro)   | PA5 / PA3 / PA4 | ADC — Y=avanti, X=laterale, Z=yaw |
| Tastino stick sinistro `BL`      | PB10 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| Tastino stick destro `BR`        | PB15 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| **EM STOP** (fungo) `EM`         | PB14 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| Button1 `B1`                     | PB11 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| Button2 `B2`                     | PB12 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| Button3 `B3`                     | PB13 | INPUT_PULLUP, NO→massa, debounce 30 ms |
| LED di vita                      | PC13 | lampeggia ogni 500 ms |

> Tutti i pulsanti sono **Normally-Open verso massa**: pull-up interno, quindi
> a riposo il pin legge HIGH e premuto legge LOW. Il firmware filtra il rimbalzo
> (finestra di stabilita' 30 ms) e pubblica **1 = premuto**.
>
> Nota storica: nel firmware precedente le etichette `LX`/`LY` (e `RX`/`RY`)
> erano **scambiate** rispetto al significato reale, e lo swap veniva compensato
> in `joy_node.py`. Ora sono corrette alla sorgente e il nodo fa un pass-through
> pulito — firmware e nodo vanno aggiornati **in coppia**.

## Elaborazione (nel firmware)
- ADC 12 bit → centro a 2048 (rimozione offset).
- `deadzone()`: zona morta ±50 conteggi, poi scala a circa **-1.0 … +1.0**.
- Tutti gli assi (incluso lo yaw LZ/RZ) normalizzati a circa **[-1, 1]**. L'eventuale
  guadagno di sensibilità dello yaw va applicato a valle (nodo teleop), non nel firmware.
- Alcuni assi invertiti via ×-1 secondo il cablaggio.

## Protocollo seriale (il "contratto" col nodo ROS2)
- **Porta / baud:** USB CDC, **57600 baud** (lato Pi: alias udev `/dev/aira_controller`,
  fisicamente `/dev/ttyACM*`).
- **Frequenza:** un messaggio ogni **20 ms** (~50 Hz).
- **Formato:** una **riga JSON** terminata da newline, con 6 assi float + 6 pulsanti int:
  ```json
  {"LY":0.00,"LZ":0.00,"LX":0.00,"RY":0.00,"RZ":0.00,"RX":0.00,"BL":0,"BR":0,"EM":0,"B1":0,"B2":0,"B3":0}
  ```
  - `BL`/`BR` = tastino stick sx/dx; `EM` = fungo emergenza; `B1..B3` = general purpose.
  - Tutti i pulsanti: **1 = premuto**, 0 = a riposo (gia' con debounce nel firmware).
- Lato Pi, `joy_node.py` fa `json.loads` e pubblica:
  - assi → due `geometry_msgs/Point`, **pass-through** (chiavi gia' corrette):
    `Point.x <- (L/R)X`, `Point.y <- (L/R)Y`, `z <- (L/R)Z`;
  - pulsanti → `std_msgs/Bool`: `left_button`, `right_button`, `emergency_stop`,
    `button_1`, `button_2`, `button_3`. I pulsanti sono opzionali (`.get(...,0)`).

## Note
- Il buffer `json_buffer[10]` è riusato per tutti i valori float: funziona perché
  ArduinoJson **copia** le stringhe da `char*`, ma è un dettaglio fragile da ricordare.
- Debounce: sampling a 50 Hz (loop 20 ms) + finestra di stabilita' 30 ms → un
  pulsante deve restare fermo ~1–2 cicli prima di essere accettato. Semplice e robusto.
- EM STOP: al momento è solo pubblicato come topic `emergency_stop`; **la gestione
  a valle (blocco motori nel teleop) è da implementare** — non è ancora una catena di
  sicurezza hardware. È voluto come **quick-stop software**.

## Flash (ST-Link/SWD) e troubleshooting USB
- **Flashare via ST-Link/SWD**, non DFU: su STM32F103 il **DFU via USB nativo non esiste**
  (bootloader di sistema F103 = solo UART/CAN). In Arduino IDE: Board `Generic STM32F1 series`
  → `BluePill F103C8`, **Upload method `STM32CubeProgrammer (SWD)`** (serve STM32CubeProgrammer
  installato + driver ST-Link). Non è il menu "Programmer".
- Dopo il flash SWD il bootloader Maple è sovrascritto; la scheda si presenta come
  **USB CDC `0483:5740`** (STMicroelectronics).
- **USB non enumera** (`dmesg`: `descriptor read error -32`, `error -71`, `unable to enumerate`):
  problema **elettrico/segnale**, non firmware. In ordine: (1) collega **diretto al Pi**, non via
  hub USB; (2) cavo dati alternativo; (3) difetto classico Blue Pill: **pull-up D+ (PA12) errato**
  (10k invece di 1.5k) → salda 1.5k tra PA12 e 3V3.
