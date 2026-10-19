#!/usr/bin/env python3
"""calibrate แขน AR4 หนึ่งตัว (เฟิร์มแวร์ ROS) นอก ROS — ทีละข้อต่อตามลำดับที่เลือก หรือทั้งหมดพร้อมกัน

  ar_calibrate.py <พอร์ต Teensy> 6 5 4 1 2 3     # ทีละข้อ ตามลำดับ (แต่ละข้อหาสวิตช์ -> ถอย -> กลับท่าพัก)
  ar_calibrate.py <พอร์ต Teensy> 5               # ข้อเดียว (ไว้ไล่ดูว่าข้อไหนพามือไปชน)
  ar_calibrate.py <พอร์ต Teensy> 456 123         # กลุ่มละหลายข้อพร้อมกัน
  ar_calibrate.py <พอร์ต Teensy> all             # 6 ข้อพร้อมกันแบบต้นฉบับ (มือที่ติดปลายแขนอาจชน)
  (arm calibrate left|right ... เรียกตัวนี้ให้)
  บน Windows ใช้ชื่อ COM ได้ตรงๆ:  python ar_calibrate.py COM9 6 5 4 1 2 3   (ต้องมี pyserial)

เลือกข้อได้ต้องใช้เฟิร์มแวร์ AR4_ROS2_seq (C:\\ar4_fw\\AR4_ROS2_seq) — ตัวนี้ถาม JV ก่อน ต้นฉบับไม่ตอบ JV และ
ถ้าได้ JC5 จะ calibrate ทั้ง 6 ข้อพร้อมกัน จึงไม่ส่งเลือกข้อให้เฟิร์มแวร์ต้นฉบับเด็ดขาด

ทำไมไม่ calibrate ใน ros2_control: ไดรเวอร์ของผู้ขายบล็อกระหว่าง calibrate และ USB ของ Teensy มักหลุดชั่วครู่
ตอนมอเตอร์หมุน — ถ้าหลุดใน ROS ทั้ง ros2_control_node ตาย ตัวนี้ทนได้: ถ้าหลุดจะรอพอร์ตกลับมาแล้วถาม JP ซ้ำ
เฟิร์มแวร์ตอบ JP ได้ก็ต่อเมื่อ calibrate กลุ่มนั้นเสร็จแล้ว — Teensy ไม่รีเซ็ตตอน USB หลุด จึงทำต่อจนจบเอง
"""
import os
import sys
import time

import serial

TIMEOUT = 300  # วินาที ต่อกลุ่ม — ทั้ง 6 ข้อพร้อมกันปกติ ~2 นาที


def open_port(path, wait=60):
    t_end = time.monotonic() + wait
    while True:
        try:
            return serial.Serial(path, 115200, timeout=0.5)
        except (serial.SerialException, OSError):
            if time.monotonic() > t_end:
                raise
            time.sleep(1.0)


def ask(ser, cmd, prefix, wait=1.5):
    """ส่งคำสั่งแล้วรอบรรทัดที่ขึ้นต้นด้วย prefix — คืนบรรทัดนั้น หรือ "" ถ้าไม่ตอบ"""
    ser.reset_input_buffer()
    ser.write(cmd)
    t_end = time.monotonic() + wait
    while time.monotonic() < t_end:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(prefix):
            return line
    return ""


def calibrate_group(ser, path, group):
    """ส่ง JC<group> แล้วรอจนเสร็จ (ทนพอร์ตหลุด) — คืน serial ที่ใช้ต่อได้"""
    label = "ทั้ง 6 ข้อพร้อมกัน" if not group else "ข้อ " + ", ".join(f"J{c}" for c in group)
    print(f"→ calibrate {label} ...", flush=True)
    ser.reset_input_buffer()
    ser.write(f"JC{group}\n".encode())
    t0 = time.monotonic()
    dropped = False
    failed = []
    while time.monotonic() - t0 < TIMEOUT:
        try:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("JC") or (dropped and line.startswith("JP")):
                print(f"  เสร็จ ({time.monotonic() - t0:.0f} วินาที)" + (" — USB เคยหลุดระหว่างทาง" if dropped else ""),
                      flush=True)
                if failed:
                    sys.exit("  ✗ ข้อ " + ", ".join(failed) + " หมุนครบระยะแล้วไม่เจอสวิตช์ — หยุดให้แล้ว ข้อนั้นยังไม่ได้ "
                             "calibrate · มักเป็นมอเตอร์ข้อนั้นหมุนกลับด้าน (ตั้งด้วย arm dir) หรือสวิตช์/สายหลุด")
                return ser
            if line.startswith("DB"):
                if " FAIL" in line and not line.startswith("DB CAL FAILED"):
                    failed.append(line.split()[1])
                    print(f"  ✗ {line[3:]}", flush=True)
                elif "limit switch OK" in line:
                    print(f"  ✓ {line.split()[1]} เจอสวิตช์แล้ว", flush=True)
                elif not line.startswith("DB CAL FAILED"):
                    print(f"  เฟิร์มแวร์: {line}", flush=True)
            if dropped and not line:
                ser.write(b"JP\n")  # ถามซ้ำ — ตอบได้เมื่อกลุ่มนี้ calibrate จบ
        except (serial.SerialException, OSError):
            if not dropped:
                print("  USB หลุดระหว่าง calibrate (ปกติตอนมอเตอร์หมุน) — รอต่อใหม่ แขนทำต่อเอง", flush=True)
            dropped = True
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)
            ser = open_port(path, wait=60)
            ser.write(b"JP\n")
    sys.exit(f"ไม่ได้คำตอบใน {TIMEOUT} วินาที ({label}) — ดูว่าแขนหยุดที่ท่าพักหรือยัง")


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    path, specs = sys.argv[1], sys.argv[2:]
    if specs == ["all"]:
        groups = [""]
    else:
        groups = []
        for spec in specs:
            if not spec or any(c not in "123456" for c in spec) or len(set(spec)) != len(spec):
                sys.exit(f"'{spec}' ไม่ถูกต้อง — ใช้เลขข้อ 1-6 เช่น 6 5 4 1 2 3 หรือ 456 123 หรือ all")
            groups.append("".join(sorted(spec)))
        done = "".join(groups)
        if len(set(done)) != len(done):
            sys.exit("มีข้อต่อซ้ำกันในลำดับ — แต่ละข้อใส่ได้ครั้งเดียว")
    # Windows: พอร์ตเป็นชื่อ COM9 ไม่ใช่ไฟล์ — os.path.exists ใช้เช็กไม่ได้ ปล่อยให้ open_port ลองเปิดเอง
    is_com = path.upper().startswith("COM") and path[3:].isdigit()
    if not is_com and not os.path.exists(path):
        sys.exit(f"ไม่เจอ {path} — เสียบ USB ของแขนและเปิดไฟแขนก่อน")
    try:
        ser = open_port(path, wait=5)
    except (serial.SerialException, OSError) as exc:
        sys.exit(f"เปิด {path} ไม่ได้ ({exc}) — เสียบ USB ของแขน เปิดไฟแขน และปิดโปรแกรมอื่นที่ใช้พอร์ตนี้อยู่")
    # เช็กก่อนว่าเป็นเฟิร์มแวร์ ROS และตอบอยู่ (ไม่ขยับแขน)
    if not ask(ser, b"JP\n", "JP"):
        sys.exit(f"Teensy ที่ {path} ไม่ตอบแบบเฟิร์มแวร์ ROS — ไม่ calibrate")
    fw = ask(ser, b"JV\n", "JV")
    if groups != [""] and fw not in ("JVseq1", "JVseq2"):
        sys.exit("เฟิร์มแวร์บนบอร์ดนี้เป็นต้นฉบับ (เลือกข้อไม่ได้ — JC5 จะหมุนทั้ง 6 ข้อ) — แฟลช AR4_ROS2_seq ก่อน "
                 "หรือใช้ all ถ้ายอมให้หมุนพร้อมกัน")
    if fw != "JVseq2":
        print("หมายเหตุ: เฟิร์มแวร์นี้ไม่ใช่ v2 — ถ้าข้อไหนหาสวิตช์ไม่เจอ บอร์ดจะค้างจนกว่าจะปิดไฟแขน", flush=True)
    missing = sorted(set("123456") - set("".join(groups))) if groups != [""] else []
    if missing:
        print("หมายเหตุ: ข้อ " + ", ".join(f"J{c}" for c in missing) + " ไม่ได้ calibrate รอบนี้ "
              "(ตำแหน่งข้อนั้นยังเป็นค่าเดิม — ถ้าเพิ่งเปิดไฟแขน ต้อง calibrate ให้ครบก่อนใช้งาน)", flush=True)
    for group in groups:
        ser = calibrate_group(ser, path, group)
    print("calibrate ครบตามที่สั่งแล้ว", flush=True)


if __name__ == "__main__":
    main()
