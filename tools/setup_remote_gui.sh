#!/usr/bin/env bash
# Remote GUI access for the dev machine (RViz / Gazebo in a browser).
#
# Run this ON the Linux machine (pikachu), once. It sets up:
#   - x11vnc  : serves an X display over VNC
#   - noVNC   : browser client (no VNC app needed on the viewing device)
#
# Two display modes:
#   attach  (default) mirrors the machine's real session on :0 -- GPU
#           accelerated, same desktop you'd see sitting at it. Requires
#           someone to be logged in graphically.
#   virtual creates a headless X display via Xvfb -- works with no
#           monitor/login, but renders in software (fine for RViz,
#           sluggish for the Gazebo GUI).
#
# Usage:
#   ./setup_remote_gui.sh install          # one-time package install
#   ./setup_remote_gui.sh attach           # serve the real :0 session
#   ./setup_remote_gui.sh virtual          # serve a headless :99 display
#   ./setup_remote_gui.sh stop
#
# Then browse to  http://<machine>:6080/vnc.html
# (Use the Tailscale hostname/IP so this works from any network; see
#  docs/remote-access.md.)

set -euo pipefail

VNC_PORT=5900
WEB_PORT=6080
VIRTUAL_DISPLAY=":99"
RESOLUTION="1920x1080x24"
PASSWD_FILE="$HOME/.vnc/passwd"

install_deps() {
  sudo apt update
  sudo apt install -y x11vnc xvfb novnc websockify
  mkdir -p "$(dirname "$PASSWD_FILE")"
  if [ ! -f "$PASSWD_FILE" ]; then
    echo "Set a VNC password (you will type this in the browser):"
    x11vnc -storepasswd "$PASSWD_FILE"
  fi
  echo "Done. Now run: $0 attach   (or: $0 virtual)"
}

start_web() {
  # noVNC's launcher name differs across distros; try both.
  if command -v novnc >/dev/null; then
    novnc --listen "$WEB_PORT" --vnc "localhost:$VNC_PORT" >/tmp/novnc.log 2>&1 &
  else
    websockify -D --web=/usr/share/novnc/ "$WEB_PORT" "localhost:$VNC_PORT" \
      >/tmp/novnc.log 2>&1 &
  fi
  echo "noVNC on http://$(hostname):$WEB_PORT/vnc.html"
}

case "${1:-}" in
  install)
    install_deps
    ;;

  attach)
    # Mirror the existing graphical session (GPU-accelerated).
    pkill -f "x11vnc" 2>/dev/null || true
    x11vnc -display :0 -rfbauth "$PASSWD_FILE" -rfbport "$VNC_PORT" \
           -forever -shared -noxdamage -bg -o /tmp/x11vnc.log
    start_web
    ;;

  virtual)
    # Headless display: no monitor or login required.
    pkill -f "Xvfb $VIRTUAL_DISPLAY" 2>/dev/null || true
    pkill -f "x11vnc" 2>/dev/null || true
    Xvfb "$VIRTUAL_DISPLAY" -screen 0 "$RESOLUTION" >/tmp/xvfb.log 2>&1 &
    sleep 2
    # A window manager makes RViz/Gazebo windows movable and resizable.
    command -v openbox >/dev/null && DISPLAY="$VIRTUAL_DISPLAY" openbox &
    x11vnc -display "$VIRTUAL_DISPLAY" -rfbauth "$PASSWD_FILE" \
           -rfbport "$VNC_PORT" -forever -shared -bg -o /tmp/x11vnc.log
    start_web
    echo
    echo "Launch GUI apps against this display, e.g.:"
    echo "  DISPLAY=$VIRTUAL_DISPLAY rviz2"
    echo "  DISPLAY=$VIRTUAL_DISPLAY ros2 launch spite_d pipeline.launch.py headless:=false ..."
    ;;

  stop)
    pkill -f x11vnc 2>/dev/null || true
    pkill -f websockify 2>/dev/null || true
    pkill -f novnc 2>/dev/null || true
    pkill -f "Xvfb $VIRTUAL_DISPLAY" 2>/dev/null || true
    echo "stopped"
    ;;

  *)
    sed -n '2,28p' "$0"
    exit 1
    ;;
esac
