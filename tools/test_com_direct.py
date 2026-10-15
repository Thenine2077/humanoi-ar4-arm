#!/usr/bin/env python3
"""ทดสอบ COM port ของ Teensy ตรงๆ บน Windows — ไม่ผ่าน WSL ไม่ผ่านสะพาน ไม่ผ่าน usbip

ใช้ตัดตัวแปรให้เหลือน้อยที่สุด: ถ้าอันนี้ยังหลุด = ปัญหาอยู่ที่บอร์ด/สาย/ไดรเวอร์
ถ้าอันนี้รอดยาว = ปัญหาอยู่ที่ชั้นบน (สะพาน / usbip / ROS)

  # โหมดอ่านอย่างเดียว (ปลอดภัยที่สุด แขนไม่ขยับแน่นอน)
  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_com_direct.py --port COM9 --seconds 60

  # โหมดเลียนแบบ ROS เต็มรูปแบบ — มี "MT" (คำสั่งขยับ) ด้วย
  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_com_direct.py --port COM9 --mt --hz 30 --seconds 120

โหมดปกติส่งแค่ "JP\\n" (3 ไบต์ ขออ่านตำแหน่ง) — ตัดตัวแปรได้ก็จริง แต่ "ไม่ครอบคลุม"
สิ่งที่ ROS ทำจริง เพราะ ARHardwareInterface ยิงสองคำสั่งต่อรอบ:
    read()  -> "JP\\n"        3 ไบต์
    write() -> "MTA...F...\\n" 67 ไบต์  <-- ตัวนี้แหละที่หนัก และทำให้สเต็ปเปอร์ทำงานจริง
--mt จึงจำลองครบทั้งคู่ ใช้ตอนต้องการรู้ว่า "อาการหลุด" มาจากตัวบอร์ด/เฟิร์มแวร์
หรือมาจากชั้นส่งข้อมูล (usbip / สะพาน TCP)

⚠️ --mt ส่งคำสั่งขยับ "ไปยังตำแหน่งปัจจุบันของแขนเอง" (อ่านด้วย JP มาก่อนแล้วเปลี่ยนหัว
   JP->MT) ตามหลักแล้วแขนจึงไม่ควรขยับ แต่มอเตอร์จะถูกสั่งงานจริง — เคลียร์พื้นที่รอบแขน
   ก่อนรันเสมอ
"""
import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("ไม่พบ pyserial — ติดตั้งด้วย:  pip install pyserial")


def read_line(ser, deadline):
    """อ่านทีละไบต์จนเจอ \\n แบบเดียวกับ TeensyDriver::receive() ของผู้ขาย"""
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return bytes(buf).decode("ascii", "replace").strip()
        if b != b"\r":
            buf += b
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0, help="ทดสอบนานกี่วินาที")
    ap.add_argument("--hz", type=float, default=10.0, help="ยิงคำสั่งกี่ครั้งต่อวินาที")
    ap.add_argument("--mt", action="store_true",
                    help="ส่ง MT (คำสั่งขยับ) ด้วย เลียนแบบ ROS เต็มรูปแบบ — มอเตอร์จะทำงานจริง")
    args = ap.parse_args()

    print(f"เปิด {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2.0)
    print("เปิดสำเร็จ — เริ่มทดสอบ (Ctrl-C เพื่อหยุด)\n")

    mt_cmd = None
    if args.mt:
        # อ่านตำแหน่งปัจจุบันก่อน แล้วเปลี่ยนหัว "JP" -> "MT" ตรงๆ ได้คำสั่งที่รูปแบบ
        # ตรงกับที่ TeensyDriver::update() สร้างเป๊ะ และสั่งไปยังตำแหน่งที่แขนอยู่อยู่แล้ว
        ser.write(b"JP\n")
        pos = read_line(ser, time.monotonic() + 2.0)
        if pos is None or not pos.startswith("JP"):
            sys.exit(f"อ่านตำแหน่งเริ่มต้นไม่ได้ (ได้: {pos!r})")
        mt_cmd = ("MT" + pos[2:] + "\n").encode("ascii")
        print(f"โหมด --mt : จะส่งคำสั่งขยับไปตำแหน่งปัจจุบัน ({len(mt_cmd)} ไบต์)")
        print(f"  {mt_cmd[:70].decode()}...\n")

    period = 1.0 / args.hz
    end = time.monotonic() + args.seconds
    latencies = []
    ok = bad = 0
    started = time.monotonic()

    try:
        while time.monotonic() < end:
            cycle_start = time.monotonic()

            # ลำดับเดียวกับ controller_manager: read() ก่อน แล้ว write()
            ser.write(b"JP\n")
            reply = read_line(ser, cycle_start + 1.0)
            if reply is not None and mt_cmd is not None:
                ser.write(mt_cmd)
                reply = read_line(ser, cycle_start + 2.0)

            elapsed = (time.monotonic() - cycle_start) * 1000.0

            if reply is None:
                bad += 1
                print(f"[{elapsed:7.1f} ms] ไม่มีคำตอบ (timeout)")
            else:
                ok += 1
                latencies.append(elapsed)
                if ok <= 3 or ok % 50 == 0:
                    print(f"[{elapsed:7.1f} ms] {reply[:70]}")

            sleep = period - (time.monotonic() - cycle_start)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nหยุดตามคำสั่ง")
    except Exception as exc:
        print(f"\n!!! พังที่วินาทีที่ {time.monotonic() - started:.1f}")
        print(f"!!! {type(exc).__name__}: {exc}")
    finally:
        try:
            ser.close()
        except OSError:
            pass

    print("\n===== สรุป =====")
    print(f"รันไป            {time.monotonic() - started:.1f} วินาที")
    print(f"ตอบกลับสำเร็จ     {ok} ครั้ง")
    print(f"ไม่ตอบ/timeout    {bad} ครั้ง")
    if latencies:
        latencies.sort()
        print(f"เวลาต่อรอบ       เฉลี่ย {statistics.mean(latencies):.1f} ms | "
              f"กลาง {latencies[len(latencies) // 2]:.1f} ms | "
              f"แย่สุด {latencies[-1]:.1f} ms")


if __name__ == "__main__":
    main()
