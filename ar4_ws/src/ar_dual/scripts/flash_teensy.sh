#!/usr/bin/env bash
# แฟลชเฟิร์มแวร์ลง Teensy ตัวเดียวที่ระบุด้วยพอร์ต (รันใน container humanoi_ros — มี teensy_loader_cli ลงไว้ด้วย apt)
#   flash_teensy.sh /dev/serial/by-path/<พอร์ต Teensy> <ไฟล์.hex>
# ทำไมต้องระบุพอร์ต: Teensy สองแขนรายงานหมายเลขเดียวกัน — สั่งให้เข้าโหมดแฟลชเฉพาะตัวที่พอร์ตนี้ (เปิดพอร์ตที่
# 134 baud) แล้ว teensy_loader_cli -w จะแฟลชตัวที่อยู่ในโหมดแฟลช ซึ่งมีตัวเดียว · ห้ามใช้ -s (รีบูตทุกตัวที่เจอ)
# ระบบแขน (ros2_control) ต้องปิดอยู่ ไม่งั้นพอร์ตไม่ว่าง
set -euo pipefail
port="$1"; hex="$2"
[ -e "$port" ] || { echo "ไม่เจอ $port"; exit 1; }
[ -f "$hex" ] || { echo "ไม่เจอ $hex"; exit 1; }
if pgrep -f ros2_control_node >/dev/null; then echo "ระบบแขนเปิดอยู่ — arm off ก่อน"; exit 1; fi

echo "สั่ง $port เข้าโหมดแฟลช ..."
python3 -c "import serial, time; s = serial.Serial('$port', 134); time.sleep(0.3); s.close()" || true
sleep 1
for attempt in 1 2 3; do
  if timeout 30 teensy_loader_cli --mcu=TEENSY41 -w -v "$hex"; then
    break
  fi
  [ $attempt -eq 3 ] && { echo "แฟลชไม่สำเร็จ"; exit 1; }
  echo "ลองใหม่ ($attempt) — บอร์ดค้างอยู่ในโหมดแฟลชแล้ว ไม่ต้องสั่งซ้ำ"
  sleep 1
done

echo "รอบอร์ดรีบูต ..."
for _ in $(seq 1 20); do [ -e "$port" ] && break; sleep 0.5; done
sleep 1
python3 - "$port" <<'EOF'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=0.5); time.sleep(0.3); s.reset_input_buffer()
for cmd in (b"JV\n", b"JP\n"):
    s.write(cmd); time.sleep(0.4); print(cmd.strip().decode(), "->", s.read(s.in_waiting or 1).decode(errors="replace").strip())
EOF
