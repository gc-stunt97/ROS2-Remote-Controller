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
| Segnale | Pin  | Note |
|---------|------|------|
| LX/LY/LZ (joystick sinistro) | PA1 / PA0 / PA2 | ADC analogici |
| RX/RY/RZ (joystick destro)   | PA5 / PA3 / PA4 | ADC analogici |
| Pulsante sinistro `butL`     | PB0  | INPUT_PULLUP |
| Pulsante destro `butR`       | PB1  | INPUT_PULLUP |
| LED di vita                  | PC13 | lampeggia ogni 500 ms |

## Elaborazione (nel firmware)
- ADC 12 bit → centro a 2048 (rimozione offset).
- `deadzone()`: zona morta ±50 conteggi, poi scala a circa **-1.0 … +1.0**.
- Yaw (LZ/RZ) moltiplicato ×2 per rendere lo sterzo più reattivo.
- Alcuni assi invertiti via ×-1 secondo il cablaggio.

## Protocollo seriale (il "contratto" col nodo ROS2)
- **Porta / baud:** USB CDC, **57600 baud** (lato Pi: `/dev/ttyACM0`).
- **Frequenza:** un messaggio ogni **20 ms** (~50 Hz).
- **Formato:** una **riga JSON** terminata da newline, con 6 assi float + 2 tastini int:
  ```json
  {"LX":0.00,"LY":0.00,"LZ":0.00,"RX":0.00,"RY":0.00,"RZ":0.00,"BL":0,"BR":0}
  ```
  - `BL`/`BR` = tastino in cima al joystick sinistro/destro: **1 = premuto**, 0 = a riposo.
- Lato Pi, `joy_node.py` fa `json.loads` e pubblica:
  - assi → due `geometry_msgs/Point`, **rimappati** alla convenzione ROS
    x=laterale(destra +), y=avanti(+): `Point.x <- (L/R)Y`, `Point.y <- (L/R)X`, `z <- (L/R)Z`;
  - tastini → due `std_msgs/Bool` (`left_button` / `right_button`).

## Note
- Il buffer `json_buffer[10]` è riusato per tutti i valori float: funziona perché
  ArduinoJson **copia** le stringhe da `char*`, ma è un dettaglio fragile da ricordare.
- I pin PB2/PB3 sono predisposti come input ma non letti: eventuali pulsanti extra futuri.
