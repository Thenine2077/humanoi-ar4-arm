#!/usr/bin/env python3
"""teach — บันทึกท่าปัจจุบันของแขนลงไฟล์ และจัดการโปรแกรม

  ros2 run ar_program teach <ชื่อท่า>                       # เก็บท่าปัจจุบัน (ทับของเดิมถ้าชื่อซ้ำ)
  ros2 run ar_program teach --list                          # ดูท่าและโปรแกรมทั้งหมด
  ros2 run ar_program teach --program pick above grasp wait=1.0 above
  ros2 run ar_program teach --delete <ชื่อท่า>
  ros2 run ar_program teach --delete-program <ชื่อโปรแกรม>

ตัวเลือก
  --file   ไฟล์ท่า (ค่าเริ่มต้น src/ar_program/config/programs.yaml)
  --topic  หัวข้อ joint_states (ค่าเริ่มต้น /joint_states, แขนจริงในชุด dual ใช้ /real/joint_states)

Jog แขนให้ได้ท่าก่อน (MotionPlanning > Joints ใน RViz แล้ว Plan & Execute) แล้วค่อยสั่ง teach
"""
import argparse
import math
import sys
import time

from ar_program import programs


def deg(values):
    return "[" + ", ".join(f"{math.degrees(v):7.1f}°" for v in values) + "]"


def read_current(topic, timeout=5.0):
    import rclpy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("ar_program_teach")
    got = {}
    node.create_subscription(
        JointState, topic,
        lambda m: got.update(dict(zip(m.name, m.position))), 10)
    end = time.monotonic() + timeout
    while time.monotonic() < end and not all(j in got for j in programs.JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not all(j in got for j in programs.JOINTS):
        missing = [j for j in programs.JOINTS if j not in got]
        sys.exit(f"ไม่ได้รับ {topic} ครบภายใน {timeout:.0f} วินาที (ขาด {missing}) — ซิม/แขนเปิดอยู่ไหม")
    return [got[j] for j in programs.JOINTS]


def parse_steps(tokens):
    steps = []
    for t in tokens:
        if t.startswith("wait="):
            steps.append({"wait": float(t[5:])})
        else:
            steps.append(t)
    return steps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="ชื่อท่าที่จะบันทึก")
    ap.add_argument("--file", default=programs.DEFAULT_FILE)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--program", nargs="+", metavar=("ชื่อ", "ขั้น"),
                    help="ตั้งโปรแกรม: ชื่อ ตามด้วยชื่อท่า/wait=วินาที ตามลำดับ")
    ap.add_argument("--delete", metavar="ชื่อท่า")
    ap.add_argument("--delete-program", metavar="ชื่อโปรแกรม")
    # ros2 run ส่ง --ros-args มาด้วยบางครั้ง ตัดทิ้งก่อน parse
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    args = ap.parse_args(argv)

    data = programs.load(args.file)

    if args.list:
        print(f"ไฟล์: {args.file}")
        print(f"\nท่า ({len(data['poses'])})   ลำดับ joint_1 .. joint_6")
        for name, vals in data["poses"].items():
            print(f"  {name:<14} {deg(vals)}")
        print(f"\nโปรแกรม ({len(data['programs'])})")
        for name, steps in data["programs"].items():
            shown = " → ".join(s if isinstance(s, str) else f"wait {s['wait']}s" for s in steps)
            print(f"  {name:<14} {shown}")
        return

    if args.delete:
        used = [p for p, steps in data["programs"].items() if args.delete in steps]
        if used:
            sys.exit(f"ลบไม่ได้ — ท่า '{args.delete}' ยังถูกใช้ในโปรแกรม: {', '.join(used)}")
        if data["poses"].pop(args.delete, None) is None:
            sys.exit(f"ไม่มีท่า '{args.delete}'")
        programs.save(args.file, data)
        print(f"ลบท่า '{args.delete}' แล้ว")
        return

    if args.delete_program:
        if data["programs"].pop(args.delete_program, None) is None:
            known = ", ".join(data["programs"]) or "(ยังไม่มี)"
            sys.exit(f"ไม่มีโปรแกรม '{args.delete_program}' — ที่มี: {known}")
        programs.save(args.file, data)
        print(f"ลบโปรแกรม '{args.delete_program}' แล้ว (ท่าในโปรแกรมยังอยู่ครบ)")
        return

    if args.program:
        name, steps = args.program[0], parse_steps(args.program[1:])
        if name.startswith("@"):
            sys.exit("ชื่อโปรแกรมห้ามขึ้นต้นด้วย @ (สงวนไว้สำหรับสั่งไปท่าเดียว เช่น @rest)")
        data["programs"][name] = steps
        try:
            resolved = programs.resolve(data, name)  # ตรวจก่อนบันทึก
        except programs.ProgramError as exc:
            sys.exit(f"บันทึกไม่ได้: {exc}")
        programs.save(args.file, data)
        print(f"บันทึกโปรแกรม '{name}' ({len(resolved)} ขั้น) ลง {args.file}")
        print(f"สั่งรัน:  ros2 topic pub --once /arm/run std_msgs/msg/String \"{{data: {name}}}\"")
        return

    if not args.name:
        ap.print_help()
        return

    values = read_current(args.topic)
    try:
        programs.check_pose(args.name, values)
    except programs.ProgramError as exc:
        sys.exit(f"ไม่บันทึก: {exc}")
    old = data["poses"].get(args.name)
    data["poses"][args.name] = values
    programs.save(args.file, data)
    if old:
        print(f"แทนที่ท่า '{args.name}'\n  เดิม {deg(old)}\n  ใหม่ {deg(values)}")
    else:
        print(f"บันทึกท่า '{args.name}' {deg(values)}")


if __name__ == "__main__":
    main()
