#!/usr/bin/env python3
"""ทดสอบว่าบอร์ดของ HMI v4.3.2 ตอบคำสั่งไหม — แบบมี timeout (ไม่ค้างเหมือน AR4.py)

  python C:\\Users\\User\\Desktop\\ar4\\tools\\test_hmi_boards.py --teensy COM9 --nano COM6

ทำไมต้องมี: AR4.py เปิดพอร์ตด้วย serial.Serial(port, baud) โดยไม่ตั้ง timeout แล้วสั่ง
ser.read() / ser.readline() รอคำตอบ ถ้าบอร์ดไม่ตอบ หน้าต่างค้าง (Not Responding) ไปตลอด
สคริปต์นี้ส่งคำสั่งชุดเดียวกับ HMI แต่รอแค่ 1.5 วินาที แล้วบอกว่าบอร์ดไหนเงียบ

คำสั่งที่ใช้ — ไม่มีคำสั่งไหนทำให้แขนขยับ:
  Teensy (9600)   : RP          ขอตำแหน่งปัจจุบัน (อ่านอย่างเดียว)
  Nano   (115200) : TMhello     echo test
                    ONX<pin> / OFX<pin>  เปิด/ปิดขา digital out (ค่าเริ่มต้นขา 14 = A0)

ต้องปิด AR4.py ก่อนรัน (พอร์ตเปิดได้ทีละโปรแกรม)
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("ไม่พบ pyserial — ติดตั้งด้วย:  pip install pyserial")

TIMEOUT = 1.5


def ask(ser, cmd, wait_line=True):
    """ส่งคำสั่งแล้วรอคำตอบไม่เกิน TIMEOUT วินาที คืน (ข้อความ, ms) หรือ (None, ms)"""
    ser.reset_input_buffer()
    t0 = time.monotonic()
    ser.write(cmd.encode())
    buf = b""
    deadline = t0 + TIMEOUT
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if not wait_line or b"\n" in buf:
                # อ่านเศษที่ตามมาอีกนิดให้ครบข้อความ
                time.sleep(0.05)
                buf += ser.read(ser.in_waiting)
                break
    ms = (time.monotonic() - t0) * 1000
    return (buf.decode("ascii", "replace").strip() or None) if buf else None, ms


def report(name, cmd, reply, ms, expect=None):
    """expect = ข้อความที่ต้องได้เป๊ะ (None = ขอแค่มีคำตอบ)"""
    shown = cmd.strip()
    if reply is None:
        print(f"  ✗ {shown:<10} -> ไม่ตอบภายใน {TIMEOUT:.1f} s   <-- AR4.py จะค้างตรงนี้")
        return False
    if expect is not None and reply != expect:
        print(f"  ✗ {shown:<10} -> {reply[:70]!r}  (ต้องได้ {expect!r} — คำสั่งไม่ได้ทำงานจริง)")
        return False
    print(f"  ✓ {shown:<10} -> {reply[:70]!r}  ({ms:.0f} ms)")
    return True


def test_teensy(port):
    print(f"\n[Teensy] {port} @ 9600")
    try:
        ser = serial.Serial(port, 9600, timeout=0.05)
    except Exception as exc:
        print(f"  ✗ เปิดพอร์ตไม่ได้: {exc}")
        return False
    with ser:
        time.sleep(0.3)
        reply, ms = ask(ser, "RP\n")
        ok = report("Teensy", "RP", reply, ms)
        if ok and not reply.startswith("A"):
            print("    ⚠ ตอบมาแต่ไม่ใช่รูปแบบตำแหน่งของ HMI (A..B..C..) — เฟิร์มแวร์อาจไม่ใช่ v4.3")
        return ok


def test_nano(port, pin):
    print(f"\n[Nano] {port} @ 115200")
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as exc:
        print(f"  ✗ เปิดพอร์ตไม่ได้: {exc}")
        return False
    with ser:
        # Nano (CH340) รีเซ็ตตัวเองทุกครั้งที่เปิดพอร์ต (สาย DTR) — bootloader กินเวลา ~1-2 วิ
        # ถ้าส่งคำสั่งเร็วกว่านี้คำสั่งจะหายไปเฉยๆ
        print("  (รอ Nano บูตหลังเปิดพอร์ต 2.5 s)")
        time.sleep(2.5)
        # ON/OF ตอบ "Done" (ไม่มีขึ้นบรรทัดใหม่) — ต้องได้คำนี้เป๊ะถึงจะนับว่าขาเปลี่ยนจริง
        r, ms = ask(ser, f"ONX{pin}\n", wait_line=False)
        ok = report("Nano", f"ONX{pin}", r, ms, expect="Done")
        if ok:
            # เฟิร์มแวร์ v2 (C:\ar4_fw\AR4_nano_inputs) กลับตรรกะให้รีเลย์ active-LOW:
            # ON = ขา LOW = รีเลย์ติด ส่วนเฟิร์มแวร์ผู้ขาย ON = ขา HIGH = รีเลย์ดับ
            # ช่องรีเลย์จริงที่ยืนยันแล้ว: ขา 14 -> รีเลย์ช่อง 8 (ลำดับกลับด้านจากเลขขา)
            print(f"    -> สั่ง ON ขา {pin} ค้าง 2 วินาที: เฟิร์มแวร์ v2 = รีเลย์ของขานี้ติด"
                  f" / เฟิร์มแวร์ผู้ขาย = ขาเป็น 5V (รีเลย์ดับ)")
            time.sleep(2.0)
        r, ms = ask(ser, f"OFX{pin}\n", wait_line=False)
        ok &= report("Nano", f"OFX{pin}", r, ms, expect="Done")
        # TM ต้องเป็นคำสั่งสุดท้ายเสมอ: เฟิร์มแวร์ Nano ของผู้ขายไม่ล้างบัฟเฟอร์หลัง TM
        # (else { inData = ""; } ผูกกับ if TM) คำสั่งถัดจาก TM จะถูกต่อท้ายแล้วตีความเป็น TM
        # อีกรอบ — ไม่ได้ทำงานจริง แต่ยังมีคำตอบกลับมา เลยหลอกให้ดูเหมือนผ่าน
        ok &= report("Nano", "TMhello", *ask(ser, "TMhello\n"), expect="hello")
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--teensy", default="COM9")
    ap.add_argument("--nano", default="COM6")
    ap.add_argument("--pin", type=int, default=14, help="ขา digital out ที่จะลอง (14-19)")
    args = ap.parse_args()
    if not 14 <= args.pin <= 19:
        sys.exit("ขา output ในเฟิร์มแวร์ Nano มีแค่ 14-19 (A0-A5)")

    t_ok = test_teensy(args.teensy)
    n_ok = test_nano(args.nano, args.pin)

    print("\n===== สรุป =====")
    print(f"  Teensy : {'ตอบปกติ' if t_ok else 'ไม่ตอบ / เปิดไม่ได้'}")
    print(f"  Nano   : {'ตอบปกติ' if n_ok else 'ไม่ตอบ / เปิดไม่ได้'}")


if __name__ == "__main__":
    main()
