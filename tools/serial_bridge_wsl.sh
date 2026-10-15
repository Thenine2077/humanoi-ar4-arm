#!/usr/bin/env bash
# ฝั่ง WSL ของสะพาน serial<->TCP: ต่อไปหา serial_bridge_win.py บน Windows
# แล้วสร้าง pty ปลอมชื่อ /dev/ttyV0 ให้ ROS ใช้แทน /dev/ttyACM0 (ไม่ต้องใช้ usbip อีก)
#
#   bash /mnt/c/Users/User/Desktop/ar4/tools/serial_bridge_wsl.sh
#
# เปิดค้างไว้ในหน้าต่างของมันเอง แล้วอีกหน้าต่างค่อยสั่ง:
#   ros2 launch ar_mirror dual_real_sim.launch.py serial_port:=/dev/ttyV0 calibrate:=False
set -euo pipefail

LINK="${LINK:-/dev/ttyV0}"
TCP_PORT="${TCP_PORT:-5555}"

if ! command -v socat >/dev/null 2>&1; then
  echo "ไม่พบ socat — ติดตั้งก่อนด้วย:  sudo apt update && sudo apt install -y socat" >&2
  exit 1
fi

# หา IP ของ Windows จากใน WSL:
#  - networkingMode ปกติ (NAT): Windows คือ default gateway ของ WSL
#  - networkingMode=mirrored: ใช้ localhost ได้เลย
HOST_IP="${HOST_IP:-$(ip route show default | awk '{print $3; exit}')}"
if [ -z "$HOST_IP" ]; then
  HOST_IP=127.0.0.1
fi
echo "[bridge] ต่อไปที่ Windows ที่ ${HOST_IP}:${TCP_PORT} ..."

# ทดสอบก่อนว่าต่อได้จริง จะได้ไม่ต้องไปงงกับ error ของ socat
if ! timeout 5 bash -c "</dev/tcp/${HOST_IP}/${TCP_PORT}" 2>/dev/null; then
  cat >&2 <<EOF
[bridge] ต่อ ${HOST_IP}:${TCP_PORT} ไม่ได้ ตรวจ 2 อย่าง:
  1) ฝั่ง Windows รัน serial_bridge_win.py ค้างไว้แล้วหรือยัง
  2) Firewall — เปิดพอร์ตด้วย PowerShell (admin):
       New-NetFirewallRule -DisplayName "AR4 serial bridge" -Direction Inbound \\
           -LocalPort ${TCP_PORT} -Protocol TCP -Action Allow
EOF
  exit 1
fi

# socat ต้องสร้าง symlink ตรงตำแหน่ง $LINK — ถ้าอยู่ใน /dev ต้องเป็น root ไม่งั้นได้
# "symlink(...): Permission denied" ตัว pty ที่สร้างจะมี perm=0666 อยู่แล้ว ROS ที่รัน
# เป็น user ธรรมดาจึงเปิดได้ปกติ แม้ socat เองจะรันด้วย sudo
SUDO=""
LINK_DIR="$(dirname "$LINK")"
if [ ! -w "$LINK_DIR" ]; then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "เขียน ${LINK_DIR} ไม่ได้ และไม่มี sudo — ลองใหม่ด้วย:  LINK=/tmp/ttyV0 bash $0" >&2
    exit 1
  fi
  echo "[bridge] ${LINK_DIR} เขียนไม่ได้ ต้องใช้ sudo (อาจถามรหัสผ่าน)"
  SUDO="sudo"
fi

$SUDO rm -f "$LINK" 2>/dev/null || true
echo "[bridge] สร้าง ${LINK} — ใช้ serial_port:=${LINK} ใน ROS (Ctrl-C เพื่อปิด)"

# raw,echo=0 = ไม่มี line discipline มายุ่งกับไบต์ (โปรโตคอล Teensy เป็น text ก็จริง
# แต่ห้ามให้ tty ไปแปลง \r\n หรือ echo กลับ) perm=0666 = ไม่ต้อง sudo chmod ทุกครั้ง
exec $SUDO socat -d -d \
  "pty,link=${LINK},raw,echo=0,perm=0666,b115200" \
  "tcp:${HOST_IP}:${TCP_PORT},nodelay,keepalive,keepidle=10,keepintvl=5,keepcnt=3"
