# camera/ — Streaming video FPV (lato CONTROLLER)

Ricevitore del video del robot: **RTP/UDP → decode → finestra sul display 7"**,
accanto a RViz e alla plancia. È un **piano dati separato da ROS2**: il video
viaggia su UDP, non su topic immagine. Il comando **pan/tilt** della testa è
un'altra cosa (va via ROS: joystick DX → teleop → servo_node).

## File
- `stream_receiver.sh` — pipeline GStreamer che riceve e mostra il video.

## Prerequisiti (sul controller)
```bash
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad gstreamer1.0-libav
```

## Uso
```bash
# 1) sul CONTROLLER (schermo 7"), avvia PRIMA il receiver:
./stream_receiver.sh 5000
# 2) sul ROBOT, avvia il sender verso l'IP del controller:
#    ./stream_sender.sh <IP_CONTROLLER> 5000
```
Trova l'IP del controller con `hostname -I`.

## Note
- **Ordine**: prima il receiver, poi il sender (UDP non ha handshake).
- **Da SSH**: la finestra deve aprirsi sul 7" → serve `export DISPLAY=:0` (lo script lo fa).
- **Decode hardware** (Pi 4, più leggero della CPU): se disponibile, sostituisci
  `avdec_h264` con `v4l2h264dec` (verifica con `gst-inspect-1.0 v4l2h264dec`).
- **Latenza**: alza/abbassa `rtpjitterbuffer latency=` (ms). Più basso = meno lag ma
  più sensibile al jitter di rete.

## Prossimo (quando funziona)
Aggiungere l'avvio del receiver alla console operatore (icona) così parte insieme a
RViz + plancia. Eventuale layout: video a tutto schermo con RViz/plancia in overlay,
o affiancati sul 7".
