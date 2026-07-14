# Avvio "a icona" del controller

Rende il controller *accendi-e-clicca*: un'icona sul desktop del Raspberry avvia
insieme il nodo joystick e la GUI (via `ros2 launch`).

Questi due file sono **template** (contengono percorsi assoluti per l'utente `giulio`);
i file "vivi" stanno nella home del Pi, non nel repo. Qui servono da backup/riferimento.

## Installazione su un controller (una volta)

### RobotHex (esapode)

```bash
# 1) script di avvio nella home (sorge ROS + imposta DISPLAY da solo)
cp desktop/robothex-controller.sh ~/robothex-controller.sh
chmod +x ~/robothex-controller.sh

# 2) icona sul desktop
cp desktop/RobotHex-Controller.desktop ~/Scrivania/RobotHex-Controller.desktop
chmod +x ~/Scrivania/RobotHex-Controller.desktop
gio set ~/Scrivania/RobotHex-Controller.desktop metadata::trusted true

# (opzionale) anche nel menu applicazioni:
cp ~/Scrivania/RobotHex-Controller.desktop ~/.local/share/applications/
```

### AIRA (base mobile)

Stessa procedura, file diversi:

```bash
cp desktop/aira-controller.sh ~/aira-controller.sh
chmod +x ~/aira-controller.sh

cp desktop/AIRA-Controller.desktop ~/Scrivania/AIRA-Controller.desktop
chmod +x ~/Scrivania/AIRA-Controller.desktop
gio set ~/Scrivania/AIRA-Controller.desktop metadata::trusted true
```

Le due icone convivono senza problemi: RobotHex avvia `joypad_controller/joypad.launch.py`,
AIRA avvia `aira_dashboard/aira.launch.py`. Non lanciarle insieme, però: si contendono la
stessa seriale del telecomando.

Al primo doppio-click, se GNOME non lo avvia, fare **tasto destro sull'icona ->
"Consenti avvio" / "Allow Launching"** (una volta sola).

## Note
- L'icona va cliccata **sullo schermo del controller** (non da SSH): è roba del desktop.
- Chiudendo la finestra della GUI si ferma anche il nodo (event handler nel launch file).
- `Terminal=false` = appare solo la GUI. Per vedere i log, mettere `Terminal=true`.
- Se un giorno servisse cambiare utente/percorso, aggiornare `Exec=` nel `.desktop`
  e `$HOME/ros2_ws` nello script.
