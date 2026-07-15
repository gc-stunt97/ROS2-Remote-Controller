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

### Modo mouse (stick destro -> cursore di Ubuntu)

Prerequisiti **una tantum** (vedi `udev/99-aira-uinput.rules` per il perché di ognuno):

```bash
sudo apt install -y python3-evdev
sudo cp udev/99-aira-uinput.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=misc   # NON basta "udevadm trigger"
sudo usermod -aG input $USER      # poi RILOGGATI: i gruppi si leggono al login
# verifica:  ls -l /dev/uinput   ->   crw-rw---- 1 root input
```

Poi, come le altre:

```bash
cp desktop/mouse-mode.sh ~/mouse-mode.sh
chmod +x ~/mouse-mode.sh

cp desktop/AIRA-Mouse.desktop ~/Scrivania/AIRA-Mouse.desktop
chmod +x ~/Scrivania/AIRA-Mouse.desktop
gio set ~/Scrivania/AIRA-Mouse.desktop metadata::trusted true

# (consigliato) parte da solo all'accensione: il telecomando ha subito il mouse
mkdir -p ~/.config/autostart
cp desktop/AIRA-Mouse.desktop ~/.config/autostart/
```

Comandi: `~/mouse-mode.sh start|stop|status`.

### Come convivono i tre modi

Le icone convivono: RobotHex avvia `joypad_controller/joypad.launch.py`, AIRA avvia
`aira_dashboard/aira.launch.py`, il modo mouse avvia `joypad_controller/mouse.launch.py`.

**La seriale del telecomando vuole un solo lettore.** ⚠️ Linux **non** lo impedisce: due
processi possono aprirla insieme senza il minimo errore e **si rubano i byte a vicenda**,
per cui arrivano righe JSON spezzate a tutti e due, entrambi le scartano, e **non funziona
nessuno dei due** -- senza che compaia un solo messaggio di errore. Sembra che gli stick
siano morti. (Successo davvero il 15/07 lanciando una plancia col modo mouse attivo.)

Per questo:
- **modo mouse ↔ plance: automatico.** Gli script delle plance chiamano `mouse-mode.sh stop`
  prima di partire e `start` alla chiusura. Non ci devi pensare: apri una plancia e il mouse
  si fa da parte, la chiudi e torna.
- **plancia ↔ plancia: a mano.** Restano da non lanciare insieme, come prima.

Al primo doppio-click, se GNOME non lo avvia, fare **tasto destro sull'icona ->
"Consenti avvio" / "Allow Launching"** (una volta sola).

## Note
- L'icona va cliccata **sullo schermo del controller** (non da SSH): è roba del desktop.
- Chiudendo la finestra della GUI si ferma anche il nodo (event handler nel launch file).
- `Terminal=false` = appare solo la GUI. Per vedere i log, mettere `Terminal=true`.
- Se un giorno servisse cambiare utente/percorso, aggiornare `Exec=` nel `.desktop`
  e `$HOME/ros2_ws` nello script.
