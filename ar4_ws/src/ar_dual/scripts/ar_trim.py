#!/usr/bin/env python3
"""แก้ค่าชดเชยศูนย์ข้อต่อ (config/zero_trims.yaml) เอง — arm trim

  ar_trim.py                          ดูค่าชดเชยทั้งสองแขน
  ar_trim.py <left|right>             ดูค่าชดเชยแขนนั้น
  ar_trim.py <left|right> <ข้อ>        ข้อนั้นตรงจริงอยู่ตอนนี้ (จัดด้วยระดับน้ำใน MoveIt) — อ่านมุมจาก /joint_states เอง
  ar_trim.py <left|right> <ข้อ> <องศา>  พิมพ์มุมที่ระบบแสดงตอนข้อนั้นตรงจริงเอง (องศา เช่น 3.2 หรือ -1.5)
  ar_trim.py <left|right> <ข้อ> =<ค่า>  ตั้งค่าชดเชยข้อนั้นตรงๆ (องศา)
  ตัวเลือก: --yes (ไม่ถามยืนยัน) · --poses keep|shift (ไม่ถามเรื่องท่าที่สอนไว้)

หลักการ: มุมที่บอร์ดรายงาน = มุมที่นับจากลิมิตสวิตช์ + ค่าชดเชย -> ข้อตรงจริงแต่อ่านได้ x° แปลว่าค่าชดเชยเกินไป x
         ค่าใหม่ = ค่าเดิม − x  (หลังเขียนลงบอร์ด ตรงนี้จะอ่านได้ 0°)
แก้แค่ไฟล์ — บอร์ดต้องเขียนด้วย arm zero ตอนระบบแขนปิด (arm trim ทำให้เองถ้าระบบปิดอยู่)
exit 0 = ปกติ (แก้หรือไม่แก้) · อื่น = ผิดพลาด · แก้ไฟล์แล้วเขียนชื่อแขนลงไฟล์ $AR_TRIM_MARK (arm.sh ดูว่าต้องเขียนบอร์ดไหม)
"""
import datetime
import math
import os
import re
import shutil
import sys

import yaml

from ar_zero import trims_file  # ar_zero.py อยู่โฟลเดอร์เดียวกัน

ARMS = {"l": "left", "left": "left", "1": "left", "ซ้าย": "left",
        "r": "right", "right": "right", "2": "right", "ขวา": "right"}
TH = {"left": "ซ้าย", "right": "ขวา"}
JOINT_TH = ["หัวไหล่ (หมุนทั้งแขน)", "ต้นแขน", "ข้อศอก", "บิดแขนท่อนล่าง", "ข้อมือพับ", "หมุนมือ"]
BIG = 20.0          # องศา — มากกว่านี้ถามซ้ำ (พิมพ์ผิด / ใส่เรเดียน / ข้อยังไม่ได้ calibrate?)
NOTHING = 0         # ดูอย่างเดียว / ยกเลิก / ค่าเท่าเดิม


def num(v):
    s = f"{v:.2f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def src_path():
    """ไฟล์จริงใน src (install เป็น symlink ชี้มา) — เขียนที่ install จะหายตอน build"""
    path = os.path.realpath(trims_file())
    if f"{os.sep}install{os.sep}" in path:
        sys.exit(f"✗ {path} อยู่ใน install (build แบบไม่ symlink) — แก้ไฟล์ใน src/ar_dual/config แทน แล้ว build")
    return path


def read_trims(path):
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    return {arm: [float(v) for v in (cfg.get(arm) or [0] * 6)] for arm in ("left", "right")}


def keep_owner(path, st):
    if hasattr(os, "chown"):  # Windows ไม่มี
        try:
            os.chown(path, st.st_uid, st.st_gid)  # container เป็น root — ไม่ให้ไฟล์กลายเป็นของ root
        except OSError:
            pass


def backup(path, stamp):
    st = os.stat(path)
    bak = f"{path}.bak-{stamp}"
    n = 2
    while os.path.exists(bak):  # สั่งซ้ำในวินาทีเดียวกัน — ไม่ทับไฟล์สำรองก่อนหน้า
        bak = f"{path}.bak-{stamp}-{n}"
        n += 1
    shutil.copy2(path, bak)
    keep_owner(bak, st)
    return bak


def write_trims(path, arm, new, note):
    """แทนบรรทัด '<arm>: [...]' (คอมเมนต์อื่นในไฟล์อยู่ครบ) + จดประวัติไว้บรรทัดบน"""
    st = os.stat(path)
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    pat = re.compile(rf"^({arm}:[ \t]*)\[[^\]\n]*\]", re.M)
    if len(pat.findall(text)) != 1:
        sys.exit(f"✗ หาบรรทัด '{arm}: [...]' ใน {path} ไม่เจอ (หรือมีหลายบรรทัด) — แก้ไฟล์เอง")
    m = pat.search(text)
    text = (text[:m.start()] + f"# {note}\n" + m.group(1) + "[" + ", ".join(num(v) for v in new) + "]"
            + text[m.end():])
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    keep_owner(tmp, st)
    os.replace(tmp, path)
    got = read_trims(path)[arm]
    if any(abs(a - b) > 0.005 for a, b in zip(got, new)):
        sys.exit(f"✗ เขียนแล้วอ่านกลับไม่ตรง ({got}) — ดูไฟล์ {path}")


def read_joint(arm, j, timeout=4.0):
    """มุมข้อ j (0-5) ตอนนี้ จาก /joint_states หรือ /<arm>/joint_states (องศา) — None ถ้าระบบแขนไม่ได้เปิด"""
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except ImportError:
        return None
    name = f"{arm}_joint_{j + 1}"
    got = []
    rclpy.init()
    node = rclpy.create_node("ar_trim_read")
    try:
        def cb(msg):
            if name in msg.name and not got:
                got.append(msg.position[msg.name.index(name)])
        subs = [node.create_subscription(JointState, t, cb, 10) for t in ("/joint_states", f"/{arm}/joint_states")]
        end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not got and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        del subs
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return math.degrees(got[0]) if got else None


def ask(prompt, choices):
    while True:
        try:
            ans = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            return None
        if ans in choices:
            return ans


def show(trims, arms):
    for arm in arms:
        print(f"แขน{TH[arm]} ({arm}): " + "  ".join(f"J{i + 1} {num(v)}" for i, v in enumerate(trims[arm])))


def main():
    args = sys.argv[1:]
    yes = "--yes" in args
    poses_mode = None
    if "--poses" in args:
        i = args.index("--poses")
        poses_mode = args[i + 1] if i + 1 < len(args) else ""
        if poses_mode not in ("keep", "shift"):
            sys.exit("--poses keep|shift")
        del args[i:i + 2]
    args = [a for a in args if a != "--yes"]
    path = src_path()
    trims = read_trims(path)
    if not args:
        show(trims, ("left", "right"))
        print(f"ไฟล์: {path}")
        return NOTHING
    arm = ARMS.get(args[0].lower())
    if arm is None or len(args) > 3 or (len(args) >= 2 and not re.fullmatch(r"[1-6]", args[1])):
        sys.exit(__doc__)
    if len(args) == 1:
        show(trims, (arm,))
        return NOTHING
    j = int(args[1]) - 1
    old = trims[arm][j]
    label = f"J{j + 1} แขน{TH[arm]} ({JOINT_TH[j]})"

    if len(args) == 3 and args[2].startswith("="):
        try:
            new_val = float(args[2][1:])
        except ValueError:
            sys.exit(f"ค่าชดเชยต้องเป็นตัวเลข (องศา) เช่น ={num(old)}")
        reading = old - new_val
        why = f"ตั้งตรง {num(old)} → {num(new_val)}"
        print(f"{label}: ค่าชดเชย {num(old)}° → {num(new_val)}°")
    else:
        if len(args) == 3:
            try:
                reading = float(args[2])
            except ValueError:
                sys.exit("มุมที่อ่านได้ต้องเป็นตัวเลข องศา เช่น 3.2 หรือ -1.5  (ตั้งค่าตรงๆ ใช้ =ค่า)")
            src = "พิมพ์เอง"
        else:
            reading = read_joint(arm, j)
            if reading is None:
                sys.exit(f"✗ อ่านมุม {arm}_joint_{j + 1} ไม่ได้ — ระบบแขนไม่ได้เปิด\n"
                         f"  เปิด arm start {arm} + arm moveit แล้วจัดข้อ {j + 1} ให้ตรงด้วยระดับน้ำ แล้วสั่งใหม่\n"
                         f"  หรือพิมพ์มุมที่อ่านได้เอง: arm trim {arm} {j + 1} <องศา>")
            src = "อ่านจากระบบแขน"
        new_val = round(old - reading, 2)
        why = f"ข้อตรงจริงอ่านได้ {reading:+.2f}°, {src}"
        print(f"{label}: ตอนข้อตรงจริง อ่านได้ {reading:+.2f}° ({src})")
        print(f"  ค่าชดเชย {num(old)}° → {num(new_val)}°   (หลังเขียนลงบอร์ด ท่าตรงนี้จะอ่านได้ 0°)")
    delta = new_val - old
    if abs(delta) < 0.005:
        print("ค่าเท่าเดิม — ไม่ต้องแก้")
        return NOTHING
    if abs(delta) > BIG:
        print(f"! เปลี่ยน {delta:+.1f}° — มากผิดปกติ (พิมพ์ผิด? ใส่เป็นเรเดียน? ข้อนั้นยังไม่ได้ calibrate?)")
        if yes or ask(f"  แน่ใจว่าถูก? พิมพ์ yes เพื่อไปต่อ: ", {"yes", "no", "n", ""}) != "yes":
            print("ยกเลิก — ไม่ได้แก้อะไร")
            return NOTHING
    elif not yes and ask("แก้ค่านี้? [y/n] ", {"y", "n", "yes", "no"}) not in ("y", "yes"):
        print("ยกเลิก — ไม่ได้แก้อะไร")
        return NOTHING

    # ท่าที่สอนไว้ของแขนนี้: คงตัวเลข หรือเลื่อนตาม (ถามก่อนเขียนอะไรทั้งนั้น)
    try:
        from ar_program import programs
    except ImportError:
        programs = None
        print("(ไม่เจอแพ็กเกจ ar_program — ไม่แตะไฟล์ท่า)")
    poses = {}
    if programs:
        data = programs.load(programs.DEFAULT_FILE)
        poses = data["poses"][arm]
    shift = False
    if poses:
        if poses_mode is None:
            print(f"\nท่าที่สอนไว้ของแขน{TH[arm]} {len(poses)} ท่า — ตอนนี้สั่งเล่นแล้วข้อ {j + 1} ไปถูกที่อยู่ไหม?")
            print(f"  y = ถูกอยู่แล้ว → เลื่อนตัวเลขข้อ {j + 1} ทุกท่า {delta:+.2f}° ให้ไปที่เดิมเหมือนก่อนแก้")
            print(f"  n = เพี้ยนอยู่ (เช่นสอนไว้ก่อนย้ายสวิตช์ / เปลี่ยนชิ้นส่วน) → คงตัวเลข ค่าใหม่จะพาไปที่ถูก")
            ans = ask("  [y/n] (ยกเลิกทั้งหมด Ctrl+C) ", {"y", "n", "yes", "no"})
            if ans is None:
                print("ยกเลิก — ไม่ได้แก้อะไร")
                return NOTHING
            shift = ans in ("y", "yes")
        else:
            shift = poses_mode == "shift"

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    bak = backup(path, stamp)
    trims[arm][j] = new_val
    write_trims(path, arm, trims[arm],
                f"{datetime.date.today()} arm trim: J{j + 1} {num(old)} -> {num(new_val)} ({why})")
    if os.environ.get("AR_TRIM_MARK"):  # arm.sh ดูไฟล์นี้ว่าต้องเขียนค่าลงบอร์ดไหม
        with open(os.environ["AR_TRIM_MARK"], "w") as f:
            f.write(arm)
    print(f"\n✓ แก้ {path} แล้ว — แขน{TH[arm]}: [{', '.join(num(v) for v in trims[arm])}]  (สำรอง: {os.path.basename(bak)})")

    if poses:
        programs._TRIMS = None  # ลิมิตของแขนเลื่อนตามค่าใหม่
        pfile = programs.DEFAULT_FILE
        if shift:
            st = os.stat(pfile)
            pbak = backup(pfile, stamp)
            for name in poses:
                poses[name] = list(poses[name])
                poses[name][j] = float(poses[name][j]) + math.radians(delta)
            programs.save(pfile, data)
            keep_owner(pfile, st)
            print(f"✓ เลื่อนข้อ {j + 1} ของท่าแขน{TH[arm]} {len(poses)} ท่า {delta:+.2f}° (สำรอง: {os.path.basename(pbak)})")
        else:
            bad = []
            for name, vals in poses.items():
                try:
                    programs.check_pose(name, vals, arm)
                except programs.ProgramError as exc:
                    bad.append(str(exc))
            print(f"✓ คงตัวเลขท่าแขน{TH[arm]} {len(poses)} ท่า")
            for b in bad:
                print(f"  ! {b} — สอนท่านี้ใหม่ (arm teach {arm} ...)")
    if arm == "left" and abs(delta) >= 0.5:
        print("! แขนซ้ายมีค่ากล้อง/ท่าจับกล่อง (arm tray-teach) ที่คำนวณจากตัวเลขข้อเดิม — ถ้าหยิบกล่องพลาด ให้ทำ arm tray-teach ใหม่")
    if path.startswith("/humanoi_ws/"):
        print("! ไฟล์นี้แก้บน Pi — repo บน PC ยังเป็นค่าเดิม (บอก Claude ให้ดึงกลับเข้า repo)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
