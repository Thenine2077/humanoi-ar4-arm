#!/usr/bin/env python3
"""broke — log an incident when an arm has a problem (belt slip, collision, stuck ...) with the previous pose and the pose it was heading to

  arm broke slipped                    log a slip — pick the related recent move (Enter = latest)
  arm broke slipped right j1 clunk     name the arm, joint and add a note (any order after the symptom)
  arm broke list                       list logged incidents
  arm broke report                     summary: which poses/paths fail most, J1 load at the time, which poses to fix

Symptoms (one English word): slipped = slip · hit = collision · stuck = stuck/not moving · dropped = arm dropped · noise = loud/vibrating
  · other = anything else (other words are logged too)
Moves come from motion_log (starts with arm start) — file locations are in ar_program/arm_log.py
"""
import re
import sys
import time

from ar_program import arm_log, programs

KINDS = {"slipped": "belt/joint slip", "hit": "collision", "stuck": "stuck/not moving", "dropped": "arm dropped",
         "noise": "loud noise/vibration", "other": "other"}
ARM_TH = {"left": "left arm", "right": "right arm"}
RECENT_S = 600   # look for the arm that moved within this window (arm not given)
SHOW = 5         # how many recent moves to offer
LOSES_POSITION = ("slipped", "hit", "dropped")
LOAD_KINDS = ("slipped", "dropped")  # related to the weight on J1 — used for the load analysis in report


def ask(prompt):
    if not sys.stdin.isatty():
        return ""
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def parse(argv):
    kind, arm, joint, note = argv[0].lower(), None, None, []
    for tok in argv[1:]:
        low = tok.lower()
        if arm is None and low in programs.ARM_ALIASES:
            arm = programs.ARM_ALIASES[low]
        elif joint is None and re.fullmatch(r"[jJ][1-6]", tok):
            joint = int(tok[1])
        else:
            note.append(tok)
    return kind, arm, joint, " ".join(note)


def pick_arm(moves, now):
    recent = [m for m in moves if now - m["ts"] < RECENT_S]
    arms = sorted({m["arm"] for m in recent})
    if len(arms) == 1:
        return arms[0]
    if len(arms) == 2:
        ans = ask("Both arms just moved — which arm? [l = left / r = right / Enter = not sure]: ").lower()
        if ans in programs.ARM_ALIASES:
            return programs.ARM_ALIASES[ans]
        return max(recent, key=lambda m: m["ts"])["arm"] if not sys.stdin.isatty() else ""
    return max(moves, key=lambda m: m["ts"])["arm"] if moves else ""


def record(argv):
    kind, arm, joint, note = parse(argv)
    now = time.time()
    if kind not in KINDS:
        print(f"(symptom '{kind}' is not in the list {', '.join(KINDS)} — logging it as typed)")
    moves = arm_log.read_moves(since=now - 7 * 86400)
    if arm is None:
        arm = pick_arm(moves, now)
    mine = [m for m in moves if not arm or m["arm"] == arm][::-1][:SHOW]
    chosen = None
    if not mine:
        print(f"No moves logged in the last 7 days{' for the ' + ARM_TH[arm] if arm else ''} — "
              "is motion_log running? (it starts with an updated arm start) · logging the incident anyway")
    else:
        print(f"Latest moves{' of the ' + ARM_TH[arm] if arm else ''} (new -> old):")
        for i, m in enumerate(mine, 1):
            res = f" result {m['result']}" if m.get("result") and m["result"] != "ok" else ""
            print(f"  {i}) {m['time'][11:]} ({arm_log.ago(m['ts'], now)})  {ARM_TH.get(m['arm'], m['arm'])}  "
                  f"{arm_log.move_text(m)}\n     {arm_log.j1_text(m)} · {m.get('src') or '?'}{res}")
        ans = ask(f"Which move did it happen on? [1-{len(mine)} · Enter = 1 · 0 = not during a move / not sure]: ")
        idx = int(ans) if ans.isdigit() else 1
        if 1 <= idx <= len(mine):
            chosen = mine[idx - 1]
            arm = arm or chosen["arm"]
    rec = {"type": "incident", "ts": round(now, 3), "time": arm_log.local_time(now), "kind": kind,
           "arm": arm or "", "joint": joint, "note": note, "move": chosen, "recent": mine}
    arm_log.append(arm_log.incidents_file(), rec)
    where = f" {ARM_TH[arm]}" if arm else ""
    what = f" — {arm_log.move_text(chosen)} ({arm_log.j1_text(chosen)})" if chosen else ""
    print(f"✓ Logged: {kind} ({KINDS.get(kind, kind)}){where}{f' joint {joint}' if joint else ''}{what}")
    print(f"  File {arm_log.incidents_file()} · summary: arm broke report")
    if kind in LOSES_POSITION and arm:
        j = joint or (1 if kind == "slipped" else None)
        which = f"joint {j}" if j else "the affected joint"
        print(f"! Motors have no encoders — after this incident ({KINDS[kind]}) the system's angle for {which} may no longer match the real arm; "
              f"calibrate it again before further use: arm recal {arm} {j or '<joint>'}")


def list_incidents():
    incs = arm_log.read_incidents()
    if not incs:
        print(f"No incidents logged yet ({arm_log.incidents_file()})")
        return
    print(f"{len(incs)} incidents logged ({arm_log.incidents_file()})")
    for r in incs:
        m = r.get("move")
        arm = ARM_TH.get(r.get("arm"), "?")
        j = f" joint {r['joint']}" if r.get("joint") else ""
        line = f"  {r['time']}  {r['kind']:<8} {arm}{j}"
        if m:
            line += f"  {arm_log.move_text(m)} · {arm_log.j1_text(m)}"
        if r.get("note"):
            line += f"  \"{r['note']}\""
        print(line)


def pose_key(m):
    """Group by target: the taught pose name, or J1-J3 angles rounded to 5° (MoveIt to an unnamed point)"""
    if m.get("to_pose"):
        return m["to_pose"]
    return "~[" + " ".join(f"{5 * round(v / 5):.0f}" for v in m["to"][:3]) + "]"


def report():
    moves = arm_log.read_moves()
    incs = arm_log.read_incidents()
    if not moves:
        print(f"No moves logged yet ({arm_log.log_dir()}) — motion_log starts with arm start")
        return
    span = f"{moves[0]['time'][:10]} to {moves[-1]['time'][:10]}"
    print(f"Summary of {len(moves)} moves ({span}) and {len(incs)} incidents")
    print("J1 load = arm weight that joint 1 has to lift, relative to calibration (100% = has swung through horizontal without slipping)\n")
    bad_ts = {}
    for r in incs:
        if r.get("move"):
            bad_ts[(r["move"]["arm"], r["move"]["ts"])] = r
    for arm in programs.ARMS:
        mv = [m for m in moves if m["arm"] == arm]
        if not mv:
            continue
        mine = [r for r in incs if r.get("arm") == arm]
        kinds = {}
        for r in mine:
            kinds[r["kind"]] = kinds.get(r["kind"], 0) + 1
        print(f"== {ARM_TH[arm]}: {len(mv)} moves · incidents "
              + (", ".join(f"{k} {n}" for k, n in kinds.items()) or "none"))
        loads = [m["j1"]["peak"] for m in mv if m.get("j1")]
        if loads:
            over = sum(1 for v in loads if round(v) > 100)
            print(f"   Peak J1 load per move: max {max(loads):.0f}% · "
                  f"over 100% {over} times ({100 * over / len(loads):.0f}%)")
        # slip / arm dropped = a J1 weight problem (collision = a path problem, not compared on load)
        slips = [r for r in mine if r["kind"] in LOAD_KINDS and r.get("move") and r["move"].get("j1")]
        if slips:
            print("   At slip / arm drop:")
            for r in slips:
                m = r["move"]
                j = m["j1"]
                way = {True: "lifting", False: "lowering"}.get(j.get("lift_at_peak"), "J1 barely turning")
                print(f"     {r['time'][5:16]} {r['kind']:<8} {arm_log.move_text(m)} — peak load {j['peak']:.0f}% "
                      f"({way} {j.get('speed_at_peak', 0):.0f}°/s) at target {j['to']:.0f}%")
            low = min(r["move"]["j1"]["peak"] for r in slips)
            risky = [m for m in mv if m.get("j1") and m["j1"]["peak"] >= low and (arm, m["ts"]) not in bad_ts]
            print(f"   Lowest load at an incident {low:.0f}% — "
                  f"other moves that reached this load without trouble: {len(risky)}")
        groups = {}
        for m in mv:
            g = groups.setdefault(pose_key(m), {"n": 0, "bad": 0, "peak": 0.0, "to": 0.0})
            g["n"] += 1
            g["bad"] += (arm, m["ts"]) in bad_ts
            if m.get("j1"):
                g["peak"] = max(g["peak"], m["j1"]["peak"])
                g["to"] = max(g["to"], m["j1"]["to"])
        print("   target             moves  incidents   max J1 load en route   at target")
        for name, g in sorted(groups.items(), key=lambda kv: (-kv[1]["bad"], -kv[1]["peak"]))[:15]:
            flag = " ✗" if g["bad"] else (" ⚠" if round(g["to"]) > 100 else "")
            print(f"   {name:<18} {g['n']:5d}  {g['bad']:9d}  {g['peak']:20.0f}%  {g['to']:9.0f}%{flag}")
        advice(arm, mv, slips, groups)
        print()


def advice(arm, mv, slips, groups):
    tips = []
    for r in slips:  # where the peak load is: at the target / at the start / along the way -> different fixes
        m, j = r["move"], r["move"]["j1"]
        if round(j["peak"]) <= 100:
            continue
        if j["to"] >= j["peak"] - 5:
            tip = (f"Re-teach pose {m.get('to_pose') or 'target ' + arm_log.angles(m['to'])} with the arm tucked closer to the J1 axis "
                   f"(it holds the pose at {j['to']:.0f}% load)")
        elif j["from"] >= j["peak"] - 5:
            tip = (f"Start pose {m.get('from_pose') or arm_log.angles(m['from'])} is already heavy ({j['from']:.0f}%) — "
                   "re-teach it tucked in, or fold the arm before leaving this pose")
        else:
            tip = (f"Path {arm_log.move_text(m)} passes through an extended-arm zone (load {j['peak']:.0f}% at J1 {j['peak_at'][0]:.0f}°) "
                   "although both ends are lighter — insert a folded-arm pose on the way, then turn J1")
        if tip not in tips:
            tips.append(tip)
    if tips:
        tips.append(f"Load of every pose: arm j1-load {arm}")
    heavy = [n for n, g in groups.items() if not g["bad"] and round(g["to"]) > 100 and not n.startswith("~")]
    if heavy:
        tips.append("No incidents yet, but these poses hold more load than during calibration: " + ", ".join(sorted(heavy)))
    if slips:
        lowering = sum(1 for r in slips if r["move"]["j1"].get("lift_at_peak") is False)
        if lowering > len(slips) / 2:
            tips.append("Most happened while the arm was lowering (the motor has to brake the weight) — check the J1 driver current and lower the acceleration")
        light = [r for r in slips if round(r["move"]["j1"]["peak"]) <= 100]
        if light:
            tips.append(f"{len(light)} happened at a load of 100% or less — suspect a loose belt / the driver "
                        "(SW4 current reduction at standstill) rather than the pose")
    if not slips and not tips:
        tips.append("No incidents to compare yet — when something happens, type arm broke slipped right away")
    for t in tips:
        print(f"   → {t}")


def main():
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return
    if argv[0] == "list":
        list_incidents()
    elif argv[0] == "report":
        report()
    else:
        record(argv)


if __name__ == "__main__":
    main()
