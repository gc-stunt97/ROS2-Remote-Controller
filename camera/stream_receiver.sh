#!/usr/bin/env bash
#
# Streaming video FPV (lato CONTROLLER): riceve H.264 RTP/UDP dal robot e lo mostra
# in una finestra sul display 7", accanto a RViz e alla plancia.
#
# Piano dati SEPARATO da ROS2 (vedi CONTROLLER_HANDBOOK / handbook robot sez. 6b).
#
# USO:
#   ./stream_receiver.sh [PORTA]      (default 5000; deve combaciare col sender)
#
# Avvia PRIMA questo, poi il sender sul robot (stream_sender.sh <IP_controller>).
set -euo pipefail

PORT="${1:-${PORT:-5000}}"
export DISPLAY="${DISPLAY:-:0}"   # finestra sullo schermo fisico del controller

echo "Ricevo H.264 RTP su udp/${PORT} -> finestra sul 7\" (DISPLAY=${DISPLAY})"

# udpsrc -> jitterbuffer (assorbe il jitter di rete) -> depay -> decode -> schermo.
# avdec_h264 = decode SOFTWARE (sempre disponibile). Per il decode HW del Pi 4 vedi README.
exec gst-launch-1.0 -v \
  udpsrc port="${PORT}" caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
  rtpjitterbuffer latency=50 ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink sync=false
