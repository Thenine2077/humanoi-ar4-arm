#!/usr/bin/env python3
"""ค่าชดเชยศูนย์ข้อต่อในบอร์ด Teensy (เฟิร์มแวร์ AR4_ROS2_seq v3 คำสั่ง JZ) เทียบกับ config/zero_trims.yaml

  ar_zero.py <พอร์ต> <left|right>            เช็กว่าบอร์ดตรงกับไฟล์ (ตรง = exit 0, ไม่ตรง = exit 1)
  ar_zero.py <พอร์ต> <left|right> --apply    เขียนค่าจากไฟล์ลงบอร์ด (เก็บใน EEPROM ปิดไฟไม่หาย)

ระบบแขน (ros2_control) ต้องปิดอยู่ — พอร์ตใช้ได้ทีละโปรแกรม และค่าที่เปลี่ยนมีผลทันทีกับตัวเลขที่บอร์ดรายงาน
ไม่ขยับแขน (แค่ถาม/ตั้งค่า)
"""
import os
import sys
import time

import serial
import yaml

TOL = 0.01  # องศา (บอร์ดเก็บละเอียด 0.01°)


def trims_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("ar_dual"), "config", "zero_trims.yaml")
    except Exception:  # รันนอก ROS (ทดสอบบน PC) — อ่านจากโฟลเดอร์ต้นฉบับข้างสคริปต์
        return os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "zero_trims.yaml")


def wanted(arm):
    with open(trims_file(), encoding="utf-8") as f:
        vals = (yaml.safe_load(f) or {}).get(arm, [0] * 6)
    vals = [float(v) for v in vals]
    if len(vals) != 6:
        sys.exit(f"zero_trims.yaml: {arm} ต้องมี 6 ค่า (J1..J6)")
    return vals


def ask(ser, cmd, prefix, wait=1.5):
    ser.reset_input_buffer()
    ser.write(cmd)
    t_end = time.monotonic() + wait
    while time.monotonic() < t_end:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    return ""


def parse(line):
    try:
        vals = [float(v) for v in line[2:].split(",")]
    except ValueError:
        return None
    return vals if len(vals) == 6 else None


def fmt(vals):
    return " ".join(f"J{i + 1} {v:+g}°" for i, v in enumerate(vals) if abs(v) > TOL) or "ไม่มี (0 ทุกข้อ)"


def main():
    args = [a for a in sys.argv[1:] if a != "--apply"]
    apply = "--apply" in sys.argv[1:]
    if len(args) != 2 or args[1] not in ("left", "right"):
        sys.exit(__doc__)
    port, arm = args
    want = wanted(arm)
    try:
        ser = serial.Serial(port, 115200, timeout=0.3, **({"exclusive": True} if os.name == "posix" else {}))
    except (serial.SerialException, OSError) as exc:
        sys.exit(f"เปิด {port} ไม่ได้ ({exc}) — ระบบแขนต้องปิดอยู่ (arm off)")
    with ser:
        time.sleep(0.2)
        fw = ask(ser, b"JV\n", "JV")
        if not fw:
            sys.exit(f"บอร์ดที่ {port} ไม่ตอบ JV — ไม่ใช่เฟิร์มแวร์ AR4_ROS2_seq หรือค้างอยู่ (ปิด-เปิดไฟแขน)")
        if fw < "JVseq3":
            if all(abs(v) <= TOL for v in want):
                print(f"แขน{arm}: เฟิร์มแวร์ {fw[2:]} ไม่มีค่าชดเชยศูนย์ — ไฟล์ก็ไม่มี ✓")
                return 0
            print(f"✗ แขน{arm}: ไฟล์ต้องการค่าชดเชย {fmt(want)} แต่บอร์ดเป็นเฟิร์มแวร์ {fw[2:]} (ไม่มี JZ)\n"
                  "  แฟลช AR4_ROS2_seq v3 ก่อน แล้ว arm zero " + arm)
            return 1
        if apply:
            line = ask(ser, ("JZ=" + ",".join(f"{v:.2f}" for v in want) + "\n").encode(), "JZ")
        else:
            line = ask(ser, b"JZ\n", "JZ")
        have = parse(line)
        if have is None:
            sys.exit(f"บอร์ดตอบ JZ ไม่ถูกรูปแบบ: {line!r}")
    if all(abs(h - w) <= TOL for h, w in zip(have, want)):
        print(f"แขน{arm}: ค่าชดเชยศูนย์ในบอร์ด {fmt(have)} ตรงกับไฟล์ ✓" + (" (เขียนแล้ว)" if apply else ""))
        return 0
    print(f"✗ แขน{arm}: ค่าชดเชยศูนย์ในบอร์ด {fmt(have)} ไม่ตรงกับไฟล์ {fmt(want)}\n"
          f"  ตัวเลขข้อต่อจะคลาดจากท่าที่สอนไว้ — เขียนค่าจากไฟล์: arm zero {arm}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
