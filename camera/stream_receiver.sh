#!/usr/bin/env bash
#
# Streaming video FPV (lato CONTROLLER): riceve RTP/UDP dal robot e lo mostra in una
# finestra sul display 7", accanto a RViz e alla plancia.
#
# Deve usare lo STESSO codec del sender (variabile CODEC, default mjpeg):
#   mjpeg -> robusto alle perdite WiFi (default)
#   h264  -> meno banda ma piu' fragile
#
# USO:
#   ./stream_receiver.sh [PORTA]          (default 5000)
#   es:  CODEC=h264 ./stream_receiver.sh 5000
#
# Avvia PRIMA questo, poi il sender sul robot.
# NB (perdite WiFi): alza il tetto del buffer UDP del kernel una volta:
#     sudo sysctl -w net.core.rmem_max=4194304
set -euo pipefail

PORT="${1:-${PORT:-5000}}"
CODEC="${CODEC:-h264}"
export DISPLAY="${DISPLAY:-:0}"   # finestra sullo schermo fisico del controller

echo "Ricevo ${CODEC} RTP su udp/${PORT} -> finestra sul 7\" (DISPLAY=${DISPLAY})"

case "${CODEC}" in
  mjpeg)
    exec gst-launch-1.0 -v \
      udpsrc port="${PORT}" buffer-size="${UDP_BUFFER:-4194304}" \
             caps="application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000" ! \
      rtpjitterbuffer latency="${LATENCY:-200}" ! \
      rtpjpegdepay ! \
      jpegdec ! \
      videoconvert ! \
      autovideosink sync=false
    ;;
  h264)
    exec gst-launch-1.0 -v \
      udpsrc port="${PORT}" buffer-size="${UDP_BUFFER:-4194304}" \
             caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
      rtpjitterbuffer latency="${LATENCY:-300}" do-lost=true ! \
      rtph264depay ! \
      h264parse ! \
      avdec_h264 ! \
      videoconvert ! \
      autovideosink sync=false
    ;;
  *)
    echo "CODEC sconosciuto: ${CODEC} (usa mjpeg | h264)" >&2
    exit 1
    ;;
esac
