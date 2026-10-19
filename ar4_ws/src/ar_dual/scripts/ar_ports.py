#!/usr/bin/env python3
"""หาว่า Teensy ของแขนไหนเสียบอยู่พอร์ตไหน โดยถามชื่อแขนจากบอร์ด — ย้ายสาย USB ไปพอร์ตไหนก็หาเจอ

  ar_ports.py                      # พิมพ์ left=/dev/serial/by-path/... right=... (arm.sh ใช้บรรทัดนี้)
  ar_ports.py --list               # ดูทุกบอร์ด: พอร์ต, เฟิร์มแวร์, ชื่อ, ข้อที่ตั้งกลับทิศ
  ar_ports.py --name <พอร์ต> left  # ตั้งชื่อแขนให้บอร์ดที่พอร์ตนั้น (ทำครั้งเดียว เก็บใน EEPROM ของบอร์ด)
  ar_ports.py --dir <พอร์ต> 000010 # ตั้งข้อที่มอเตอร์หมุนกลับด้าน J1..J6 (1 = กลับ) — แขนที่ประกอบข้อนั้นกลับด้าน
  ar_ports.py --identify           # ไล่ทีละบอร์ด: ข้อ 6 หมุนไป-กลับ 10° ให้ดูว่าแขนไหนขยับ แล้วตอบ l/r ตั้งชื่อให้เลย
  ar_ports.py --wiggle <พอร์ต|left|right>  # หมุนข้อ 6 ไป-กลับ ของบอร์ดเดียว (เช็กว่าชื่อนั้นคือแขนไหนจริง)

ทำไมต้องถามบอร์ด: Teensy สองแขนรายงานหมายเลข USB เดียวกัน (13173040) /dev/serial/by-id จึงแยกไม่ได้
และ /dev/serial/by-path ผูกกับพอร์ต — ย้ายสายแล้วหาไม่เจอ · เฟิร์มแวร์ AR4_ROS2_seq v2 ตอบชื่อแขน (JN) ได้
บอร์ดที่กำลัง calibrate หรือถูกระบบแขนเปิดใช้อยู่จะไม่ตอบ — ให้ปิดระบบแขนก่อน (arm off)

--identify / --wiggle แทนการถอดสายดูว่าบอร์ดไหนหาย (ห้ามถอด/เสียบ USB แขนขณะไฟแขนเปิด — Pi 5 เคยเสีย):
ส่ง MT แบบเดียวกับที่ ROS ส่ง แต่เปลี่ยนแค่ข้อ 6 จากตำแหน่งปัจจุบัน แล้วกลับที่เดิม — ความเร็ว/ความเร่ง
เป็นค่าในเฟิร์มแวร์ (ROS ไม่เคยส่ง SS) · ไม่ต้อง calibrate ก่อน เพราะ MT ขยับตามผลต่างจากตำแหน่งที่บอร์ดนับอยู่
"""
import glob
import os
import re
import sys
import time

import serial

AXES = "ABCDEF"      # ชื่อข้อในข้อความ JP / MT ของเฟิร์มแวร์ (J1..J6)
WIGGLE_JOINT = 5     # ข้อ 6 (ข้อมือหมุน) — หมุนนิดเดียวไม่พาแขนหรือมือไปชนอะไร
WIGGLE_DEG = 10.0
ARM_ALIASES = {"l": "left", "left": "left", "1": "left", "ซ้าย": "left",
               "r": "right", "right": "right", "2": "right", "ขวา": "right"}


def by_path(dev):
    """ชื่อ /dev/serial/by-path ของ /dev/ttyACMx (คงที่ระหว่างรอบนี้ — ใช้ส่งให้ ros2_control)"""
    for link in sorted(glob.glob("/dev/serial/by-path/*")):
        if "usbv2" not in link and os.path.realpath(link) == os.path.realpath(dev):
            return link
    return dev


def ask(ser, cmd, prefix, wait=1.0):
    ser.reset_input_buffer()
    ser.write(cmd.encode() + b"\n")
    t_end = time.monotonic() + wait
    while time.monotonic() < t_end:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line[len(prefix):]
    return None


def probe(dev):
    """{port, fw, name, dir, error} ของบอร์ดหนึ่งตัว — อ่านอย่างเดียว ไม่ขยับแขน"""
    info = {"port": by_path(dev), "fw": None, "name": None, "dir": None, "error": None}
    try:
        with serial.Serial(dev, 115200, timeout=0.3) as ser:
            time.sleep(0.2)
            if ask(ser, "JP", "JP") is None:
                info["error"] = "ไม่ตอบ (กำลัง calibrate / ระบบแขนใช้อยู่ / ไม่ใช่เฟิร์มแวร์ ROS)"
                return info
            info["fw"] = ask(ser, "JV", "JV") or "original"
            if info["fw"] == "seq2":
                info["name"] = ask(ser, "JN", "JN")
                info["dir"] = ask(ser, "JD", "JD")
    except (serial.SerialException, OSError) as exc:
        info["error"] = f"เปิดพอร์ตไม่ได้ ({exc.__class__.__name__} — ระบบแขนเปิดใช้อยู่?)"
    return info


def teensy_ports():
    """พอร์ต Teensy ทุกตัว (VID 16c0) — ไม่แตะ CH340 ของ Nano/Mega"""
    out = []
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        vid = ""
        sysdev = os.path.realpath(f"/sys/class/tty/{os.path.basename(dev)}/device/..")
        try:
            vid = open(os.path.join(sysdev, "idVendor")).read().strip()
        except OSError:
            pass
        if vid == "16c0":
            out.append(dev)
    return out


def read_joints(ser):
    """มุม J1..J6 (องศา) จาก JP — None ถ้าบอร์ดไม่ตอบ"""
    reply = ask(ser, "JP", "JP")
    vals = dict(re.findall(r"([A-F])(-?[0-9.]+)", reply or ""))
    return [float(vals[a]) for a in AXES] if len(vals) == len(AXES) else None


def move_to(ser, target, joint, timeout=8.0):
    """ส่ง MT (รูปแบบเดียวกับไดรเวอร์ ROS) แล้วรอจนข้อ joint ถึงเป้า — คืน True ถ้าถึงในเวลา"""
    ask(ser, "MT" + "".join(f"{a}{v:.6f}" for a, v in zip(AXES, target)), "JP")
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        time.sleep(0.1)
        now = read_joints(ser)
        if now and abs(now[joint] - target[joint]) < 0.3:
            return True
    return False


def wiggle(port, times=2):
    """หมุนข้อ 6 ไป WIGGLE_DEG แล้วกลับที่เดิม times รอบ (หมุนเข้าหา 0° เสมอ ไม่ไปทางสุดระยะ)"""
    with serial.Serial(port, 115200, timeout=0.3, exclusive=True) as ser:
        time.sleep(0.2)
        start = read_joints(ser)
        if start is None:
            raise RuntimeError("บอร์ดไม่ตอบ JP (กำลัง calibrate / ระบบแขนใช้อยู่?)")
        out = list(start)
        out[WIGGLE_JOINT] += -WIGGLE_DEG if start[WIGGLE_JOINT] > 0 else WIGGLE_DEG
        try:
            for _ in range(times):
                if not (move_to(ser, out, WIGGLE_JOINT) and move_to(ser, start, WIGGLE_JOINT)):
                    move_to(ser, start, WIGGLE_JOINT)
                    return False
        except KeyboardInterrupt:
            move_to(ser, start, WIGGLE_JOINT)   # Ctrl+C กลางทาง — พาข้อ 6 กลับที่เดิมก่อนออก
            raise
    return True


def identify():
    """ไล่ทีละบอร์ด: หมุนข้อ 6 ให้ดู แล้วถามว่าแขนไหน -> ตั้งชื่อ (JN) เก็บในบอร์ด"""
    infos = [probe(dev) for dev in teensy_ports()]
    if not infos:
        sys.exit("ไม่เจอ Teensy เลย — เปิดไฟแขนก่อน (สาย USB ไม่ต้องถอด/ย้าย)")
    print(f"เจอบอร์ด {len(infos)} ตัว — จะหมุนข้อ 6 (ข้อมือ) ไป-กลับ {WIGGLE_DEG:.0f}° ทีละบอร์ด ให้ดูว่าแขนไหนขยับ")
    print("ดูให้รอบมือ/ข้อมือทั้งสองแขนว่าง และเปิดไฟแขนไว้ (ไม่ต้อง calibrate ก่อน)")
    input("พร้อมแล้วกด Enter (ยกเลิก Ctrl+C) ")
    for n, info in enumerate(infos, 1):
        head = f"[{n}/{len(infos)}] {info['port']}"
        if info["error"]:
            print(f"{head}: ข้าม — {info['error']}")
            continue
        if info["fw"] != "seq2":
            print(f"{head}: ข้าม — เฟิร์มแวร์ {info['fw']} เก็บชื่อแขนไม่ได้ (แฟลช AR4_ROS2_seq v2 ก่อน)")
            continue
        print(f"{head}  ชื่อตอนนี้: {info['name'] or '(ยังไม่ตั้ง)'} — ดูข้อ 6 ...")
        while True:
            try:
                if not wiggle(info["port"]):
                    print("  ! ข้อ 6 ไม่ถึงเป้าในเวลา — สั่งให้กลับที่เดิมแล้ว")
            except (RuntimeError, serial.SerialException, OSError) as exc:
                print(f"  ! ข้ามบอร์ดนี้ — {exc}")
                break
            ans = input("  แขนไหนขยับ?  l = ซ้าย · r = ขวา · a = ขยับอีกที (ไม่เห็นขยับ = ไฟแขนปิดอยู่?) · s = ข้าม : ")
            ans = ans.strip().lower()
            if ans in ARM_ALIASES:
                set_value(info["port"], "JN", ARM_ALIASES[ans])
                break
            if ans in ("s", "skip"):
                break
    print()
    print_ports([probe(dev) for dev in teensy_ports()])


def print_ports(infos):
    """บรรทัด left=... right=... (arm.sh อ่าน) + เตือนบอร์ดที่ยังใช้ไม่ได้ทาง stderr"""
    found = {}
    for i in infos:
        if i["name"]:
            if i["name"] in found:
                sys.exit(f"มีบอร์ดชื่อ '{i['name']}' สองตัว ({found[i['name']]}, {i['port']}) — ตั้งชื่อใหม่ (arm identify หรือ --name)")
            found[i["name"]] = i["port"]
    print(" ".join(f"{k}={v}" for k, v in sorted(found.items())))
    problems = [i for i in infos if not i["name"]]
    for i in problems:
        print(f"! {i['port']}: " + (i["error"] or "ยังไม่ได้ตั้งชื่อแขน (arm identify หรือ ar_ports.py --identify)"),
              file=sys.stderr)


def set_value(port, cmd, value):
    with serial.Serial(port, 115200, timeout=0.3) as ser:
        time.sleep(0.2)
        if ask(ser, "JV", "JV") != "seq2":
            sys.exit("บอร์ดนี้ไม่ใช่เฟิร์มแวร์ AR4_ROS2_seq v2 — แฟลชก่อน")
        reply = ask(ser, f"{cmd}={value}", cmd)
        print(f"{port}: {cmd} = {reply}")


def main():
    args = sys.argv[1:]
    if args[:1] == ["--name"] and len(args) == 3:
        return set_value(args[1], "JN", args[2])
    if args[:1] == ["--dir"] and len(args) == 3:
        if len(args[2]) != 6 or set(args[2]) - {"0", "1"}:
            sys.exit("ใส่ 6 หลัก 0/1 ของ J1..J6 เช่น 000010 = J5 กลับทิศ")
        return set_value(args[1], "JD", args[2])
    if args == ["--identify"]:
        return identify()
    if args[:1] == ["--wiggle"] and len(args) == 2:
        port = args[1]
        if not os.path.exists(port):
            name = ARM_ALIASES.get(port.lower(), port)
            port = next((i["port"] for i in map(probe, teensy_ports()) if i["name"] == name), None)
            if port is None:
                sys.exit(f"ไม่เจอบอร์ดชื่อ '{name}' — ดู: ar_ports.py --list")
        print(f"{port}: หมุนข้อ 6 ไป-กลับ {WIGGLE_DEG:.0f}° ...")
        try:
            if not wiggle(port):
                sys.exit("ข้อ 6 ไม่ถึงเป้าในเวลา — สั่งให้กลับที่เดิมแล้ว")
        except (RuntimeError, serial.SerialException, OSError) as exc:
            sys.exit(f"{port}: {exc}")
        print("เสร็จ — ไม่เห็นข้อ 6 ขยับ = ไฟแขนปิดอยู่ หรือเป็นบอร์ดของอีกแขน")
        return
    infos = [probe(dev) for dev in teensy_ports()]
    if args[:1] == ["--list"]:
        if not infos:
            print("ไม่เจอ Teensy เลย — เสียบ USB และเปิดไฟแขน")
        for i in infos:
            print(f"{i['port']}: " + (i["error"] or
                  f"เฟิร์มแวร์ {i['fw']} · ชื่อ {i['name'] or '(ยังไม่ตั้ง)'} · ข้อกลับทิศ {i['dir'] or '-'}"))
        return
    print_ports(infos)


if __name__ == "__main__":
    main()
