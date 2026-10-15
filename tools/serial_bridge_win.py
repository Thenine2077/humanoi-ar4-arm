#!/usr/bin/env python3
"""สะพาน serial <-> TCP ฝั่ง Windows สำหรับข้าม usbip

ทำไมต้องมี
----------
usbip ทำงานได้แต่ช้าและไม่ทน: ar_hardware_interface ยิง serial round-trip แบบ blocking
สองครั้งต่อรอบ control loop (read -> "JP\\n" รอคำตอบ, write -> "MT...\\n" รอคำตอบ)
ผ่าน usbip วัดได้ 25 ms + 43 ms ต่อรอบ ทั้งที่แขนยังไม่ขยับ แล้วหลุดภายใน ~1 นาที
เทียบกับเปิด COM ตรงบน Windows ที่วัดได้ 1.9 ms ต่อรอบ (ดู test_com_direct.py --mt)

สะพานนี้เลี่ยง usbip ทั้งหมด: Windows เปิด COM port ตรงๆ (ไม่มี USB forwarding)
แล้ว stream ไบต์ดิบผ่าน TCP ให้ WSL ฝั่ง WSL ใช้ socat ทำ pty ปลอมขึ้นมา
ROS จึงเห็นเป็น /dev/ttyV0 ธรรมดา ไม่ต้องแก้โค้ดผู้ขายเลยแม้แต่บรรทัดเดียว

⚠️ ออกแบบเป็น "เธรดเดียว" โดยตั้งใจ — ห้ามแยกเธรดอ่าน/เธรดเขียน
   pyserial ไม่ thread-safe: เวอร์ชันแรกของไฟล์นี้แยกเป็น 2 เธรด (serial->tcp และ
   tcp->serial) พอ ROS เริ่มส่งข้อมูลจริง เธรดหนึ่งค้างอยู่ใน ser.read() ขณะอีกเธรด
   เรียก ser.write() พร้อมกัน แล้วพังด้วย
       ClearCommError failed (PermissionError(13, 'The device does not recognize
       the command.', winerror 22 = ERROR_BAD_COMMAND))
   ตามด้วย COM หายไปจาก Windows ทั้งพอร์ต (FileNotFoundError ตอนเปิดใหม่)
   ซึ่ง "หน้าตาเหมือนอุปกรณ์ถูกถอด" มาก จนหลงคิดว่าเป็นที่บอร์ดอยู่นาน
   พิสูจน์แล้วว่าไม่ใช่: test_com_direct.py --mt --hz 30 (เธรดเดียว เปิด COM ตรง)
   วิ่งได้ 3572 รอบ × 2 ครั้ง ไม่พลาดเลยสักรอบ ด้วย traffic แบบเดียวกันเป๊ะ

วิธีใช้
-------
  ฝั่ง Windows (PowerShell ธรรมดา ไม่ต้อง admin):
      pip install pyserial
      python C:\\Users\\User\\Desktop\\ar4\\tools\\serial_bridge_win.py --port COM9

  ฝั่ง WSL (อีกหน้าต่าง):
      bash /mnt/c/Users/User/Desktop/ar4/tools/serial_bridge_wsl.sh
      # -> สร้าง /dev/ttyV0 ให้ แล้วสั่ง ROS ด้วย serial_port:=/dev/ttyV0

หมายเหตุ: ถ้า WSL ต่อไม่ติด มักติดที่ Windows Firewall — เปิดพอร์ตด้วย (PowerShell admin):
  New-NetFirewallRule -DisplayName "AR4 serial bridge" -Direction Inbound `
      -LocalPort 5555 -Protocol TCP -Action Allow
"""
import argparse
import select
import socket
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("ไม่พบ pyserial — ติดตั้งด้วย:  pip install pyserial")


def serve_client(conn, addr, ser, idle_s):
    """ส่งไบต์สองทางด้วยเธรดเดียว คืน True ถ้า serial ยังใช้ได้อยู่

    เช็ค socket ด้วย select แบบ timeout=0 (poll ล้วน ไม่รอ) แล้วเช็ค ser.in_waiting
    ทั้งการอ่านและการเขียน serial จึงเกิดในเธรดนี้เธรดเดียวเสมอ ไม่มีทางซ้อนกัน

    ทำไมต้อง timeout=0 + time.sleep แทนที่จะให้ select รอเอง:
    Windows มี timer granularity ~15.6 ms การสั่ง select ให้รอ 1 ms จึงถูกปัดขึ้นเป็น
    สิบกว่า ms ทุกครั้ง วัดจริงได้ 23.6 ms ต่อรอบ (เทียบกับ 1.9 ms ตอนเปิด COM ตรง)
    ส่วน time.sleep() ของ Python 3.11+ บน Windows ใช้ high-resolution waitable timer
    จึงหลับได้ละเอียดระดับต่ำกว่ามิลลิวินาทีจริง
    และถ้ารอบไหน "มีงานทำ" จะไม่หลับเลย เพื่อให้เก็บไบต์ที่เหลือของ message ต่อทันที
    """
    print(f"[bridge] WSL ต่อเข้ามาจาก {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # ทิ้งข้อมูลค้างท่อจาก session ก่อนหน้า ไม่งั้น driver จะ parse เจอเศษ message เก่า
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    serial_ok = True
    try:
        closed = False
        while not closed:
            busy = False

            # ---- TCP -> serial ----
            readable, _, _ = select.select([conn], [], [], 0)
            if readable:
                data = conn.recv(4096)
                if not data:
                    print(f"[bridge] {addr} ปิดการเชื่อมต่อ")
                    closed = True
                else:
                    ser.write(data)
                    busy = True

            # ---- serial -> TCP ----
            waiting = ser.in_waiting
            if waiting:
                conn.sendall(ser.read(waiting))
                busy = True

            if not busy and not closed:
                time.sleep(idle_s)
    except OSError as exc:
        # แยกให้ออกว่าใครพัง: ถ้า COM ยังเปิดอยู่ได้แปลว่าเป็นฝั่ง TCP (client หลุดเฉยๆ)
        serial_ok = ser.is_open and _serial_alive(ser)
        who = "serial" if not serial_ok else "TCP"
        print(f"[bridge] {addr} จบการทำงาน ({who}): {exc}")
    finally:
        try:
            conn.close()
        except OSError:
            pass

    print("[bridge] รอ client ใหม่")
    return serial_ok


def _serial_alive(ser):
    """เช็คว่า COM ยังตอบ IOCTL อยู่ไหม (in_waiting เรียก ClearCommError ข้างใน)"""
    try:
        ser.in_waiting
        return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM9", help="COM port ของ Teensy (ดูใน Device Manager)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--tcp-port", type=int, default=5555)
    ap.add_argument("--idle-us", type=float, default=200.0,
                    help="หลับกี่ไมโครวินาทีตอนไม่มีข้อมูล (ยิ่งน้อยยิ่งหน่วงต่ำ แลกกับ CPU)")
    args = ap.parse_args()

    def open_serial():
        """เปิด COM port วนรอจนกว่าจะได้ (เผื่อ Teensy ถูกถอด/reboot กลางทาง)"""
        while True:
            try:
                # timeout=0 -> read() ไม่บล็อกเลย เราอ่านเฉพาะจำนวนที่ in_waiting บอกอยู่แล้ว
                s = serial.Serial(args.port, args.baud, timeout=0, write_timeout=2.0)
                print(f"[bridge] เปิด {args.port} @ {args.baud} แล้ว")
                return s
            except OSError as exc:
                print(f"[bridge] เปิด {args.port} ไม่ได้ ({exc}) — ลองใหม่ใน 2 วิ")
                time.sleep(2.0)

    ser = open_serial()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.listen, args.tcp_port))
    srv.listen(1)
    print(f"[bridge] ฟังอยู่ที่ {args.listen}:{args.tcp_port} — รอ WSL ต่อเข้ามา (Ctrl-C เพื่อออก)")

    idle_s = args.idle_us / 1_000_000.0
    try:
        while True:
            conn, addr = srv.accept()
            if not serve_client(conn, addr, ser, idle_s):
                print("[bridge] serial มีปัญหา — ปิดแล้วเปิด COM ใหม่")
                try:
                    ser.close()
                except OSError:
                    pass
                ser = open_serial()
    except KeyboardInterrupt:
        print("\n[bridge] ปิดตามคำสั่ง")
    finally:
        srv.close()
        try:
            ser.close()
        except OSError:
            pass


if __name__ == "__main__":
    main()
