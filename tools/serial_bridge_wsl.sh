#!/usr/bin/env bash
# WSL side of the serial<->TCP bridge: connects to serial_bridge_win.py on Windows
# and creates a fake pty named /dev/ttyV0 for ROS to use instead of /dev/ttyACM0 (no more usbip)
#
#   bash /mnt/c/Users/User/Desktop/ar4/tools/serial_bridge_wsl.sh
#
# Leave it running in its own terminal window, then in another window run:
#   ros2 launch ar_mirror dual_real_sim.launch.py serial_port:=/dev/ttyV0 calibrate:=False
set -euo pipefail

LINK="${LINK:-/dev/ttyV0}"
TCP_PORT="${TCP_PORT:-5555}"

if ! command -v socat >/dev/null 2>&1; then
  echo "socat not found — install it first with:  sudo apt update && sudo apt install -y socat" >&2
  exit 1
fi

# Find the Windows IP from inside WSL:
#  - default networkingMode (NAT): Windows is WSL's default gateway
#  - networkingMode=mirrored: just use localhost
HOST_IP="${HOST_IP:-$(ip route show default | awk '{print $3; exit}')}"
if [ -z "$HOST_IP" ]; then
  HOST_IP=127.0.0.1
fi
echo "[bridge] connecting to Windows at ${HOST_IP}:${TCP_PORT} ..."

# Test the connection first, so we don't get confused by socat's errors
if ! timeout 5 bash -c "</dev/tcp/${HOST_IP}/${TCP_PORT}" 2>/dev/null; then
  cat >&2 <<EOF
[bridge] can't connect to ${HOST_IP}:${TCP_PORT} — check 2 things:
  1) Is serial_bridge_win.py already running on the Windows side?
  2) Firewall — open the port with PowerShell (admin):
       New-NetFirewallRule -DisplayName "AR4 serial bridge" -Direction Inbound \\
           -LocalPort ${TCP_PORT} -Protocol TCP -Action Allow
EOF
  exit 1
fi

# socat must create a symlink at $LINK — if that's in /dev it needs root, otherwise you get
# "symlink(...): Permission denied". The pty it creates already has perm=0666, so ROS running
# as a normal user can open it even though socat itself runs with sudo
SUDO=""
LINK_DIR="$(dirname "$LINK")"
if [ ! -w "$LINK_DIR" ]; then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "can't write ${LINK_DIR} and there is no sudo — try again with:  LINK=/tmp/ttyV0 bash $0" >&2
    exit 1
  fi
  echo "[bridge] ${LINK_DIR} is not writable, need sudo (may ask for a password)"
  SUDO="sudo"
fi

$SUDO rm -f "$LINK" 2>/dev/null || true
echo "[bridge] creating ${LINK} — use serial_port:=${LINK} in ROS (Ctrl-C to close)"

# raw,echo=0 = no line discipline touching the bytes (the Teensy protocol is text,
# but the tty must not convert \r\n or echo back) perm=0666 = no sudo chmod every time
exec $SUDO socat -d -d \
  "pty,link=${LINK},raw,echo=0,perm=0666,b115200" \
  "tcp:${HOST_IP}:${TCP_PORT},nodelay,keepalive,keepidle=10,keepintvl=5,keepcnt=3"
