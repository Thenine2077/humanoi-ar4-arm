#!/usr/bin/env python3
"""teach — save an arm's current pose to the file, and manage programs (two arms: left = arm 1, right = arm 2)

  ros2 run ar_program teach <pose name> --arm left         # save the current pose of the left arm (overwrites an existing name)
  ros2 run ar_program teach --list                         # show the poses of every arm and all programs
  ros2 run ar_program teach --show pick2                   # show a program as a table: step | left | right
  ros2 run ar_program teach --program pick2 act1/act1 act2/- act3/act2 -/act3 wait=1.0
  ros2 run ar_program teach --program grab act1/- -/-/rock act2/-/paper     # with hand gestures
  ros2 run ar_program teach --delete <pose name> --arm right
  ros2 run ar_program teach --delete-program <program name>
  ros2 run ar_program teach --j1-load [--arm right]           # J1 load of every pose (arm j1-load)

Program steps: <left arm pose>/<right arm pose>[/<left hand>[/<right hand>]] — use - (or still) if it doesn't move in that step
  hand gestures: rock paper scissors point like windup victory nonono relax · wait=seconds = wait
  arm and hand in the same step move together — to let the arm arrive before the hand closes, split the step: act1/- then -/-/rock

Options
  --file    pose file (default src/ar_program/config/programs.yaml)
  --topic   joint_states topic (default /joint_states; the real arm in the dual_real_sim setup uses /real/joint_states)
  --layout  dual (default, ar_dual two arms) | single (old single arm, joints joint_N — for the sim on the PC)

Jog the arm into the pose first (RViz: pick the arm in Planning Group -> Joints tab, or drag the ball -> Plan & Execute)
then run teach — after saving it reports the J1 load (j1_load.py); over 100% of the calibrate load = risk of J1 belt slip
"""
import argparse
import math
import sys
import time

from ar_program import j1_load, programs

ARM_TH = {"left": "left", "right": "right"}


def deg(values):
    return "[" + ", ".join(f"{math.degrees(v):7.1f}°" for v in values) + "]"


def read_current(topic, names, timeout=5.0):
    """Current pose (rad) + robot model from /robot_description (for the J1 load calculation — unavailable = None)"""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("ar_program_teach")
    got, urdf = {}, []
    node.create_subscription(
        JointState, topic,
        lambda m: got.update(dict(zip(m.name, m.position))), 10)
    node.create_subscription(String, "/robot_description", lambda m: urdf.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.monotonic() + timeout
    while time.monotonic() < end and not all(j in got for j in names):
        rclpy.spin_once(node, timeout_sec=0.1)
    end = time.monotonic() + 1.0
    while time.monotonic() < end and not urdf:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not all(j in got for j in names):
        missing = [j for j in names if j not in got]
        sys.exit(f"Did not receive all of {topic} within {timeout:.0f} s (missing {missing}) — is the arm system running?")
    return [got[j] for j in names], (urdf[0] if urdf else None)


def j1_note(loader, arm, values):
    """J1 load message for the pose just saved"""
    pct = loader.percent(arm, values)
    sweep = loader.sweep_percent(arm, values)
    if not j1_load.over(pct):
        line = f"J1 load: {pct:.0f}% of the calibrate load ✓"
    else:
        line = (f"⚠ J1 load: {pct:.0f}% of the calibrate load (over 100%) — in this pose the arm reaches far from the J1 axis, "
                f"risk of J1 belt slip\n  Try pulling the arm in (lower J2 / bend J3 toward the body), then teach the same name again · "
                f"see all poses: arm j1-load {arm}")
    if j1_load.over(sweep):
        line += (f"\n  Rotating J1 far with the arm in this shape raises the load to {sweep:.0f}% — "
                 "fold the arm first, then rotate J1; it is lighter")
    return line


def print_j1_table(loader, data, arms):
    print("J1 load of the taught poses — 100% = during calibrate (J1 swinging through horizontal has never slipped)")
    print("Masses from the model (AR3 values + hand), comparable as % only · hold = at that pose · "
          "rotate J1 = if J1 swings its whole range with the arm in this shape")
    for arm in arms:
        poses = data["poses"][arm]
        if not poses or not loader.has_arm(arm):
            continue
        rows = sorted(((loader.percent(arm, v), loader.sweep_percent(arm, v), n, v) for n, v in poses.items()),
                      key=lambda r: -r[0])
        w = max(len(n) for n in poses)
        print(f"\nArm: {ARM_TH[arm]} ({len(rows)} poses)")
        print("  " + "pose".ljust(w) + "     J1     J2     J3      hold  rotate J1")
        for pct, sweep, name, v in rows:
            print(f"  {name:<{w}} {math.degrees(v[0]):6.0f} {math.degrees(v[1]):6.0f} {math.degrees(v[2]):6.0f}"
                  f"   {pct:6.0f}%   {sweep:6.0f}%   {j1_load.label(pct)}")


def print_table(data, name):
    rows = programs.table(data, name)
    hands = any(r[3] not in ("", "-") or r[4] not in ("", "-") for r in rows)  # has hand gestures = also show the hand columns
    cols = ["Left arm (1)", "Right arm (2)"] + (["Left hand", "Right hand"] if hands else [])
    widths = [max([len(c)] + [len(r[k + 1]) for r in rows]) for k, c in enumerate(cols)]
    print(f"Program '{name}' ({len(rows)} steps)")
    print(f"  {'Step':>4}  " + "  ".join(f"{c:<{w}}" for c, w in zip(cols, widths)).rstrip())
    for r in rows:
        print(f"  {r[0]:>4}  " + "  ".join(f"{v:<{w}}" for v, w in zip(r[1:1 + len(cols)], widths)).rstrip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="name of the pose to save")
    ap.add_argument("--arm", help="arm to teach / delete a pose from: left (1) or right (2)")
    ap.add_argument("--file", default=programs.DEFAULT_FILE)
    ap.add_argument("--topic", default="/joint_states")
    ap.add_argument("--layout", default="dual", choices=["dual", "single"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--show", metavar="PROGRAM")
    ap.add_argument("--program", nargs="+", metavar=("NAME", "STEP"),
                    help="set a program: name followed by steps <left>/<right> or wait=seconds, in order")
    ap.add_argument("--delete", metavar="POSE")
    ap.add_argument("--delete-program", metavar="PROGRAM")
    ap.add_argument("--hand-names", action="store_true", help="print the hand gesture names that can be commanded (built-in + self-taught)")
    ap.add_argument("--j1-load", action="store_true", help="J1 load of every pose (--arm = one arm only)")
    # ros2 run sometimes passes --ros-args too; cut it off before parsing
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    # --program steps such as "-/act3" start with -, which argparse would take for an option — split them off first
    program_tokens = None
    if "--program" in argv:
        i = argv.index("--program")
        argv, program_tokens = argv[:i], argv[i + 1:]
        if not program_tokens:
            sys.exit("usage: --program <name> <left arm>/<right arm>[/<left hand>[/<right hand>]] ... "
                     "(e.g. grab act1/- -/-/rock act2/-/paper wait=1.0)")
    args = ap.parse_args(argv)
    if program_tokens is not None:
        args.program = program_tokens

    data = programs.load(args.file)

    def need_arm():
        if args.layout == "single":
            return "left"
        if not args.arm:
            sys.exit("Specify the arm with --arm left (arm 1) or --arm right (arm 2)")
        try:
            return programs.arm_of(args.arm)
        except programs.ProgramError as exc:
            sys.exit(str(exc))

    if args.hand_names:
        print("\n".join(sorted(programs.hand_names(data))))
        return

    if args.j1_load:
        arms = [programs.arm_of(args.arm)] if args.arm else list(programs.ARMS)
        print_j1_table(j1_load.J1Load(j1_load.urdf_text()), data, arms)
        return

    if args.list:
        print(f"File: {args.file}")
        for arm in programs.ARMS:
            poses = data["poses"][arm]
            print(f"\nPoses of the {ARM_TH[arm]} arm ({len(poses)})   order joint_1 .. joint_6")
            for name, vals in poses.items():
                print(f"  {name:<14} {deg(vals)}")
        for side, key in programs.HAND_POSE_KEYS.items():
            taught = data[key]
            if not taught and side == "left":
                continue
            print(f"\nSelf-taught hand gestures, {ARM_TH[side]} hand set ({len(taught)})   arm hand-teach · same name as a built-in = replaces it")
            for name, vals in taught.items():
                mark = " (replaces built-in)" if name in programs.HAND_GESTURES else ""
                print(f"  {name:<14} [" + ", ".join(f"{v:+.0f}" for v in vals) + f"]{mark}")
        print(f"\nPrograms ({len(data['programs'])})   view step by step with --show <name>")
        for name in data["programs"]:
            shown = " → ".join(programs.cli_step(r) for r in programs.table(data, name))
            print(f"  {name:<14} {shown}")
        return

    if args.show:
        if args.show not in data["programs"]:
            sys.exit(f"Program '{args.show}' not found — available: {', '.join(data['programs']) or '(none yet)'}")
        print_table(data, args.show)
        return

    if args.delete:
        arm = need_arm()
        used = [p for p, steps in data["programs"].items()
                if any((programs.step_arms(s) or {}).get(arm) == args.delete for s in steps)]
        if used:
            sys.exit(f"Cannot delete — pose '{args.delete}' of the {ARM_TH[arm]} arm is still used in programs: {', '.join(used)}")
        if data["poses"][arm].pop(args.delete, None) is None:
            sys.exit(f"The {ARM_TH[arm]} arm has no pose '{args.delete}'")
        programs.save(args.file, data)
        print(f"Deleted pose '{args.delete}' of the {ARM_TH[arm]} arm")
        return

    if args.delete_program:
        if data["programs"].pop(args.delete_program, None) is None:
            known = ", ".join(data["programs"]) or "(none yet)"
            sys.exit(f"Program '{args.delete_program}' not found — available: {known}")
        programs.save(args.file, data)
        print(f"Deleted program '{args.delete_program}' (its poses are all kept)")
        return

    if args.program:
        name = args.program[0]
        if name.startswith("@"):
            sys.exit("Program names must not start with @ (reserved for going to a single pose, e.g. @rest)")
        try:
            if args.layout == "single":  # single arm: old-style bare pose names
                steps = [{"wait": float(t[5:])} if t.startswith("wait=") else t for t in args.program[1:]]
            else:
                steps = [programs.parse_cli_step(t, programs.hand_names(data)) for t in args.program[1:]]
            data["programs"][name] = steps
            resolved = programs.resolve(data, name)  # check before saving
        except (programs.ProgramError, ValueError) as exc:
            sys.exit(f"Cannot save: {exc}")
        programs.save(args.file, data)
        print(f"Saved program '{name}' ({len(resolved)} steps) to {args.file}\n")
        print_table(data, name)
        return

    if not args.name:
        ap.print_help()
        return

    arm = need_arm()
    values, urdf = read_current(args.topic, programs.joint_names(arm, args.layout))
    try:
        programs.check_pose(args.name, values, arm)
    except programs.ProgramError as exc:
        sys.exit(f"Not saved: {exc}")
    old = data["poses"][arm].get(args.name)
    data["poses"][arm][args.name] = values
    programs.save(args.file, data)
    if old:
        print(f"Replaced pose '{args.name}' of the {ARM_TH[arm]} arm\n  old {deg(old)}\n  new {deg(values)}")
    else:
        print(f"Saved pose '{args.name}' of the {ARM_TH[arm]} arm {deg(values)}")
    if args.layout == "dual":
        try:
            loader = j1_load.J1Load(urdf or j1_load.urdf_text())
            if old:
                print(f"  previous J1 load {loader.percent(arm, old):.0f}%")
            print(j1_note(loader, arm, values))
        except Exception as exc:  # failing to calculate is no reason not to save
            print(f"(could not calculate J1 load: {exc})")


if __name__ == "__main__":
    main()
