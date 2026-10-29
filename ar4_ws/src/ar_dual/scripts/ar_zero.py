#!/usr/bin/env python3
"""ค่าชดเชยศูนย์ข้อต่อในบอร์ด Teensy (เฟิร์มแวร์ AR4_ROS2_seq v3 คำสั่ง JZ) เทียบกับ config/zero_trims.yaml

  ar_zero.py <พอร์ต> <left|right>            เช็กว่าบอร์ดตรงกับไฟล์ (ตรง = exit 0, ไม่ตรง = exit 1)
  ar_zero.py <พอร์ต> <left|right> --apply    เขียนค่าจากไฟล์ลงบอร์ด (เก็บใน EEPROM ปิดไฟไม่หาย)

ค่าชดเชยตรงแล้วตั้งเพดานความเร็วมอเตอร์จาก config/joint_speed.yaml ลงบอร์ดด้วย (คำสั่ง SS — หายเมื่อบอร์ดรีบูต
arm start / recal / reconnect จึงเรียกตัวนี้ก่อนเปิดตัวคุมทุกครั้ง)
ระบบแขน (ros2_control) ต้องปิดอยู่ — พอร์ตใช้ได้ทีละโปรแกรม และค่าที่เปลี่ยนมีผลทันทีกับตัวเลขที่บอร์ดรายงาน
ไม่ขยับแขน (แค่ถาม/ตั้งค่า)
"""
import os
import sys
import time

import serial
import yaml

TOL = 0.01  # องศา (บอร์ดเก็บละเอียด 0.01°)


def config_file(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("ar_dual"), "config", name)
    except Exception:  # รันนอก ROS (ทดสอบบน PC) — อ่านจากโฟลเดอร์ต้นฉบับข้างสคริปต์
        return os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", name)


def trims_file():
    return config_file("zero_trims.yaml")


def speed_command():
    """คำสั่ง SS จาก config/joint_speed.yaml -> (bytes, [speed J1..J6]) · ไฟล์ไม่มี = (None, None)

    เฟิร์มแวร์แยกค่าด้วยตัวอักษร A..L: J1 ความเร็ว A ความเร่ง B, J2 C/D ... J6 K/L (ทศนิยมธรรมดา ห้ามมี e)"""
    path = config_file("joint_speed.yaml")
    if not os.path.exists(path):
        return None, None
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    speed = [float(v) for v in cfg.get("speed", [])]
    accel = [float(v) for v in cfg.get("accel", [])]
    if len(speed) != 6 or len(accel) != 6 or min(speed + accel) <= 0:
        sys.exit(f"{path}: speed / accel ต้องมีค่าบวก 6 ค่า (J1..J6)")
    body = "".join(f"{'ABCDEFGHIJKL'[2 * i]}{s:.3f}{'ABCDEFGHIJKL'[2 * i + 1]}{a:.3f}"
                   for i, (s, a) in enumerate(zip(speed, accel)))
    return f"SS{body}\n".encode(), speed


def set_speed(ser, arm=""):
    """ตั้งเพดานความเร็วในบอร์ด (ไม่ขยับแขน — บอร์ดตอบตำแหน่ง JP กลับมา) คืน True ถ้าบอร์ดรับ"""
    cmd, speed = speed_command()
    if cmd is None:
        return True
    if not ask(ser, cmd, "JP"):
        print(f"⚠ แขน{arm}: บอร์ดไม่ตอบคำสั่งตั้งความเร็ว (SS) — ข้อ 1 ยังใช้เพดานเดิมของเฟิร์มแวร์ 30°/s", flush=True)
        return False
    slow = " ".join(f"J{i + 1} {v:g}°/s" for i, v in enumerate(speed) if v < 30) or "ทุกข้อ 30°/s"
    print(f"แขน{arm}: ตั้งเพดานความเร็วมอเตอร์ในบอร์ดแล้ว — {slow} (config/joint_speed.yaml) ✓", flush=True)
    return True


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
        name = ask(ser, b"JN\n", "JN")[2:].strip()
        if fw >= "JVseq2" and name != arm:  # กันพอร์ตที่จำไว้ผิดแขน (สองบอร์ดหมายเลข USB ซ้ำกัน)
            print(f"✗ บอร์ดที่ {port} ชื่อ '{name or 'ไม่มีชื่อ'}' ไม่ใช่แขน {arm} — ดู arm ports (ตอนระบบแขนปิด)")
            return 1
        if fw < "JVseq3":
            if all(abs(v) <= TOL for v in want):
                print(f"แขน{arm}: เฟิร์มแวร์ {fw[2:]} ไม่มีค่าชดเชยศูนย์ — ไฟล์ก็ไม่มี ✓")
                set_speed(ser, arm)
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
            set_speed(ser, arm)
            return 0
    print(f"✗ แขน{arm}: ค่าชดเชยศูนย์ในบอร์ด {fmt(have)} ไม่ตรงกับไฟล์ {fmt(want)}\n"
          f"  ตัวเลขข้อต่อจะคลาดจากท่าที่สอนไว้ — เขียนค่าจากไฟล์: arm zero {arm}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
