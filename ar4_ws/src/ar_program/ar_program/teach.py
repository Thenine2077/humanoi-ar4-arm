#!/usr/bin/env python3
"""teach — บันทึกท่าปัจจุบันของแขนลงไฟล์ และจัดการโปรแกรม (สองแขน: left = แขน 1, right = แขน 2)

  ros2 run ar_program teach <ชื่อท่า> --arm left           # เก็บท่าปัจจุบันของแขนซ้าย (ทับของเดิมถ้าชื่อซ้ำ)
  ros2 run ar_program teach --list                         # ดูท่าของทุกแขนและโปรแกรมทั้งหมด
  ros2 run ar_program teach --show pick2                   # ดูโปรแกรมเป็นตาราง ขั้น | ซ้าย | ขวา
  ros2 run ar_program teach --program pick2 act1/act1 act2/- act3/act2 -/act3 wait=1.0
  ros2 run ar_program teach --program grab act1/- -/-/rock act2/-/paper     # มีท่ามือ
  ros2 run ar_program teach --delete <ชื่อท่า> --arm right
  ros2 run ar_program teach --delete-program <ชื่อโปรแกรม>

ขั้นของโปรแกรม: <ท่าแขนซ้าย>/<ท่าแขนขวา>[/<มือซ้าย>[/<มือขวา>]] — ใช้ - (หรือ still) ถ้าไม่ขยับในขั้นนั้น
  ท่ามือ: rock paper scissors point like windup victory nonono relax · wait=วินาที = รอ
  แขนกับมือในขั้นเดียวกันขยับพร้อมกัน — อยากให้แขนถึงก่อนแล้วมือค่อยกำ ให้แยกขั้น: act1/- แล้ว -/-/rock

ตัวเลือก
  --file    ไฟล์ท่า (ค่าเริ่มต้น src/ar_program/config/programs.yaml)
  --topic   หัวข้อ joint_states (ค่าเริ่มต้น /joint_states, แขนจริงในชุด dual_real_sim ใช้ /real/joint_states)
  --layout  dual (ค่าเริ่ม ar_dual สองแขน) | single (แขนเดี่ยวแบบเดิม ข้อต่อ joint_N — ใช้กับซิมบน PC)

Jog แขนให้ได้ท่าก่อน (RViz: Planning Group เลือกแขน -> แท็บ Joints หรือลากลูกบอล -> Plan & Execute)
แล้วค่อยสั่ง teach
"""
import argparse
import math
import sys
import time

from ar_program import programs

ARM_TH = {"left": "ซ้าย", "right": "ขวา"}


def deg(values):
    return "[" + ", ".join(f"{math.degrees(v):7.1f}°" for v in values) + "]"


def read_current(topic, names, timeout=5.0):
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("ar_program_teach")
    got = {}
    node.create_subscription(
        JointState, topic,
        lambda m: got.update(dict(zip(m.name, m.position))), 10)
    end = time.monotonic() + timeout
    while time.monotonic() < end and not all(j in got for j in names):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not all(j in got for j in names):
        missing = [j for j in names if j not in got]
        sys.exit(f"ไม่ได้รับ {topic} ครบภายใน {timeout:.0f} วินาที (ขาด {missing}) — ระบบแขนเปิดอยู่ไหม")
    return [got[j] for j in names]


def print_table(data, name):
    rows = programs.table(data, name)
    hands = any(r[3] not in ("", "-") or r[4] not in ("", "-") for r in rows)  # มีท่ามือ = แสดงคอลัมน์มือด้วย
    cols = ["แขนซ้าย (1)", "แขนขวา (2)"] + (["มือซ้าย", "มือขวา"] if hands else [])
    widths = [max([len(c)] + [len(r[k + 1]) for r in rows]) for k, c in enumerate(cols)]
    print(f"โปรแกรม '{name}' ({len(rows)} ขั้น)")
    print(f"  {'ขั้น':>4}  " + "  ".join(f"{c:<{w}}" for c, w in zip(cols, widths)).rstrip())
    for r in rows:
        print(f"  {r[0]:>4}  " + "  ".join(f"{v:<{w}}" for v, w in zip(r[1:1 + len(cols)], widths)).rstrip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="ชื่อท่าที่จะบันทึก")
    ap.add_argument("--arm", help="แขนที่จะ teach / ลบท่า: left (1) หรือ right (2)")
    ap.add_argument("--file", default=programs.DEFAULT_FILE)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--layout", default="dual", choices=["dual", "single"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--show", metavar="ชื่อโปรแกรม")
    ap.add_argument("--program", nargs="+", metavar=("ชื่อ", "ขั้น"),
                    help="ตั้งโปรแกรม: ชื่อ ตามด้วยขั้น <ซ้าย>/<ขวา> หรือ wait=วินาที ตามลำดับ")
    ap.add_argument("--delete", metavar="ชื่อท่า")
    ap.add_argument("--delete-program", metavar="ชื่อโปรแกรม")
    ap.add_argument("--hand-names", action="store_true", help="พิมพ์ชื่อท่ามือที่สั่งได้ (ท่าในตัว + ท่าที่สอนเอง)")
    # ros2 run ส่ง --ros-args มาด้วยบางครั้ง ตัดทิ้งก่อน parse
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    # ขั้นของ --program เช่น "-/act3" ขึ้นต้นด้วย - ซึ่ง argparse จะเข้าใจผิดว่าเป็นตัวเลือก — แยกออกมาเองก่อน
    program_tokens = None
    if "--program" in argv:
        i = argv.index("--program")
        argv, program_tokens = argv[:i], argv[i + 1:]
        if not program_tokens:
            sys.exit("ใช้: --program <ชื่อ> <แขนซ้าย>/<แขนขวา>[/<มือซ้าย>[/<มือขวา>]] ... "
                     "(เช่น grab act1/- -/-/rock act2/-/paper wait=1.0)")
    args = ap.parse_args(argv)
    if program_tokens is not None:
        args.program = program_tokens

    data = programs.load(args.file)

    def need_arm():
        if args.layout == "single":
            return "left"
        if not args.arm:
            sys.exit("ระบุแขนด้วย --arm left (แขน 1) หรือ --arm right (แขน 2)")
        try:
            return programs.arm_of(args.arm)
        except programs.ProgramError as exc:
            sys.exit(str(exc))

    if args.hand_names:
        print("\n".join(sorted(programs.hand_names(data))))
        return

    if args.list:
        print(f"ไฟล์: {args.file}")
        for arm in programs.ARMS:
            poses = data["poses"][arm]
            print(f"\nท่าแขน{ARM_TH[arm]} ({len(poses)})   ลำดับ joint_1 .. joint_6")
            for name, vals in poses.items():
                print(f"  {name:<14} {deg(vals)}")
        for side, key in programs.HAND_POSE_KEYS.items():
            taught = data[key]
            if not taught and side == "left":
                continue
            print(f"\nท่ามือที่สอนเอง ชุดมือ{ARM_TH[side]} ({len(taught)})   arm hand-teach · ชื่อซ้ำท่าในตัว = ใช้แทน")
            for name, vals in taught.items():
                mark = " (แทนท่าในตัว)" if name in programs.HAND_GESTURES else ""
                print(f"  {name:<14} [" + ", ".join(f"{v:+.0f}" for v in vals) + f"]{mark}")
        print(f"\nโปรแกรม ({len(data['programs'])})   ดูทีละขั้นด้วย --show <ชื่อ>")
        for name in data["programs"]:
            shown = " → ".join(programs.cli_step(r) for r in programs.table(data, name))
            print(f"  {name:<14} {shown}")
        return

    if args.show:
        if args.show not in data["programs"]:
            sys.exit(f"ไม่มีโปรแกรม '{args.show}' — ที่มี: {', '.join(data['programs']) or '(ยังไม่มี)'}")
        print_table(data, args.show)
        return

    if args.delete:
        arm = need_arm()
        used = [p for p, steps in data["programs"].items()
                if any((programs.step_arms(s) or {}).get(arm) == args.delete for s in steps)]
        if used:
            sys.exit(f"ลบไม่ได้ — ท่า '{args.delete}' ของแขน{ARM_TH[arm]} ยังถูกใช้ในโปรแกรม: {', '.join(used)}")
        if data["poses"][arm].pop(args.delete, None) is None:
            sys.exit(f"แขน{ARM_TH[arm]}ไม่มีท่า '{args.delete}'")
        programs.save(args.file, data)
        print(f"ลบท่า '{args.delete}' ของแขน{ARM_TH[arm]}แล้ว")
        return

    if args.delete_program:
        if data["programs"].pop(args.delete_program, None) is None:
            known = ", ".join(data["programs"]) or "(ยังไม่มี)"
            sys.exit(f"ไม่มีโปรแกรม '{args.delete_program}' — ที่มี: {known}")
        programs.save(args.file, data)
        print(f"ลบโปรแกรม '{args.delete_program}' แล้ว (ท่าในโปรแกรมยังอยู่ครบ)")
        return

    if args.program:
        name = args.program[0]
        if name.startswith("@"):
            sys.exit("ชื่อโปรแกรมห้ามขึ้นต้นด้วย @ (สงวนไว้สำหรับสั่งไปท่าเดียว เช่น @rest)")
        try:
            if args.layout == "single":  # แขนเดี่ยว: ชื่อท่าเปล่าๆ แบบเดิม
                steps = [{"wait": float(t[5:])} if t.startswith("wait=") else t for t in args.program[1:]]
            else:
                steps = [programs.parse_cli_step(t, programs.hand_names(data)) for t in args.program[1:]]
            data["programs"][name] = steps
            resolved = programs.resolve(data, name)  # ตรวจก่อนบันทึก
        except (programs.ProgramError, ValueError) as exc:
            sys.exit(f"บันทึกไม่ได้: {exc}")
        programs.save(args.file, data)
        print(f"บันทึกโปรแกรม '{name}' ({len(resolved)} ขั้น) ลง {args.file}\n")
        print_table(data, name)
        return

    if not args.name:
        ap.print_help()
        return

    arm = need_arm()
    values = read_current(args.topic, programs.joint_names(arm, args.layout))
    try:
        programs.check_pose(args.name, values, arm)
    except programs.ProgramError as exc:
        sys.exit(f"ไม่บันทึก: {exc}")
    old = data["poses"][arm].get(args.name)
    data["poses"][arm][args.name] = values
    programs.save(args.file, data)
    if old:
        print(f"แทนที่ท่า '{args.name}' ของแขน{ARM_TH[arm]}\n  เดิม {deg(old)}\n  ใหม่ {deg(values)}")
    else:
        print(f"บันทึกท่า '{args.name}' ของแขน{ARM_TH[arm]} {deg(values)}")


if __name__ == "__main__":
    main()
