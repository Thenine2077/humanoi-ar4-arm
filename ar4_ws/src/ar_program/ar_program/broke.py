#!/usr/bin/env python3
"""broke — จดเหตุการณ์ตอนแขนมีปัญหา (สายพานรูด ชน ค้าง ...) พร้อมท่าก่อนหน้าและท่าที่กำลังไป

  arm broke slipped                    จดว่ารูด — เลือกการขยับล่าสุดที่เกี่ยว (Enter = ครั้งล่าสุด)
  arm broke slipped right j1 ดังแก๊ก    ระบุแขน ข้อ และบันทึกเพิ่มได้ (ลำดับไหนก็ได้หลังชื่ออาการ)
  arm broke list                       ดูเหตุการณ์ที่จดไว้
  arm broke report                     สรุป: ท่า/ทางไหนเกิดเรื่องบ่อย ภาระ J1 ตอนนั้น ควรแก้ท่าไหน

อาการ (ภาษาอังกฤษ คำเดียว): slipped รูด · hit ชน · stuck ค้าง/ไม่ขยับ · dropped แขนตก · noise เสียงดัง/สั่น
  · other อื่นๆ (คำอื่นก็จดได้)
การขยับมาจาก motion_log (เปิดเองพร้อม arm start) — ไฟล์อยู่ที่ ar_program/arm_log.py บอก
"""
import re
import sys
import time

from ar_program import arm_log, programs

KINDS = {"slipped": "สายพาน/ข้อรูด", "hit": "ชน", "stuck": "ค้าง/ไม่ขยับ", "dropped": "แขนตก",
         "noise": "เสียงดัง/สั่น", "other": "อื่นๆ"}
ARM_TH = {"left": "ซ้าย", "right": "ขวา"}
RECENT_S = 600   # หาแขนที่เพิ่งขยับในช่วงนี้ (ไม่ได้บอกแขน)
SHOW = 5         # แสดงการขยับล่าสุดกี่ครั้งให้เลือก
LOSES_POSITION = ("slipped", "hit", "dropped")
LOAD_KINDS = ("slipped", "dropped")  # เกี่ยวกับน้ำหนักที่ J1 — ใช้วิเคราะห์ภาระใน report


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
        ans = ask("ทั้งสองแขนเพิ่งขยับ — แขนไหน? [l = ซ้าย / r = ขวา / Enter = ทั้งคู่ไม่แน่ใจ]: ").lower()
        if ans in programs.ARM_ALIASES:
            return programs.ARM_ALIASES[ans]
        return max(recent, key=lambda m: m["ts"])["arm"] if not sys.stdin.isatty() else ""
    return max(moves, key=lambda m: m["ts"])["arm"] if moves else ""


def record(argv):
    kind, arm, joint, note = parse(argv)
    now = time.time()
    if kind not in KINDS:
        print(f"(อาการ '{kind}' ไม่อยู่ในรายการ {', '.join(KINDS)} — จดให้ตามที่พิมพ์)")
    moves = arm_log.read_moves(since=now - 7 * 86400)
    if arm is None:
        arm = pick_arm(moves, now)
    mine = [m for m in moves if not arm or m["arm"] == arm][::-1][:SHOW]
    chosen = None
    if not mine:
        print(f"ไม่มีการขยับที่จดไว้ใน 7 วัน{' ของแขน' + ARM_TH[arm] if arm else ''} — "
              "motion_log เปิดอยู่ไหม (เปิดเองกับ arm start ที่อัปเดตแล้ว) · จดเหตุการณ์ไว้ก่อน")
    else:
        print(f"การขยับล่าสุด{' ของแขน' + ARM_TH[arm] if arm else ''} (ใหม่ -> เก่า):")
        for i, m in enumerate(mine, 1):
            res = f" ผล {m['result']}" if m.get("result") and m["result"] != "ok" else ""
            print(f"  {i}) {m['time'][11:]} ({arm_log.ago(m['ts'], now)})  {ARM_TH.get(m['arm'], m['arm'])}  "
                  f"{arm_log.move_text(m)}\n     {arm_log.j1_text(m)} · {m.get('src') or '?'}{res}")
        ans = ask(f"เกิดตอนครั้งไหน? [1-{len(mine)} · Enter = 1 · 0 = ไม่ใช่ตอนขยับ/ไม่แน่ใจ]: ")
        idx = int(ans) if ans.isdigit() else 1
        if 1 <= idx <= len(mine):
            chosen = mine[idx - 1]
            arm = arm or chosen["arm"]
    rec = {"type": "incident", "ts": round(now, 3), "time": arm_log.local_time(now), "kind": kind,
           "arm": arm or "", "joint": joint, "note": note, "move": chosen, "recent": mine}
    arm_log.append(arm_log.incidents_file(), rec)
    where = f" แขน{ARM_TH[arm]}" if arm else ""
    what = f" — {arm_log.move_text(chosen)} ({arm_log.j1_text(chosen)})" if chosen else ""
    print(f"✓ จดแล้ว: {kind} ({KINDS.get(kind, kind)}){where}{f' ข้อ {joint}' if joint else ''}{what}")
    print(f"  ไฟล์ {arm_log.incidents_file()} · ดูสรุป: arm broke report")
    if kind in LOSES_POSITION and arm:
        j = joint or (1 if kind == "slipped" else None)
        which = f"ข้อ {j}" if j else "ข้อที่โดน"
        print(f"! มอเตอร์ไม่มี encoder — หลัง{KINDS[kind]} ตัวเลขของ{which} ในระบบอาจไม่ตรงกับแขนจริงแล้ว "
              f"calibrate ใหม่ก่อนใช้ต่อ: arm recal {arm} {j or '<ข้อ>'}")


def list_incidents():
    incs = arm_log.read_incidents()
    if not incs:
        print(f"ยังไม่มีเหตุการณ์ที่จดไว้ ({arm_log.incidents_file()})")
        return
    print(f"เหตุการณ์ที่จดไว้ {len(incs)} ครั้ง ({arm_log.incidents_file()})")
    for r in incs:
        m = r.get("move")
        arm = ARM_TH.get(r.get("arm"), "?")
        j = f" ข้อ {r['joint']}" if r.get("joint") else ""
        line = f"  {r['time']}  {r['kind']:<8} แขน{arm}{j}"
        if m:
            line += f"  {arm_log.move_text(m)} · {arm_log.j1_text(m)}"
        if r.get("note"):
            line += f"  \"{r['note']}\""
        print(line)


def pose_key(m):
    """จัดกลุ่มตามปลายทาง: ชื่อท่าที่สอนไว้ หรือมุม J1-J3 ปัดทีละ 5° (MoveIt ไปจุดที่ไม่ได้ตั้งชื่อ)"""
    if m.get("to_pose"):
        return m["to_pose"]
    return "~[" + " ".join(f"{5 * round(v / 5):.0f}" for v in m["to"][:3]) + "]"


def report():
    moves = arm_log.read_moves()
    incs = arm_log.read_incidents()
    if not moves:
        print(f"ยังไม่มีการขยับที่จดไว้ ({arm_log.log_dir()}) — motion_log เปิดพร้อม arm start")
        return
    span = f"{moves[0]['time'][:10]} ถึง {moves[-1]['time'][:10]}"
    print(f"สรุปจาก {len(moves)} การขยับ ({span}) และเหตุการณ์ {len(incs)} ครั้ง")
    print("ภาระ J1 = น้ำหนักแขนที่ข้อ 1 ต้องยก เทียบตอน calibrate (100% = เคยหมุนผ่านแนวนอนไม่รูด)\n")
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
        print(f"== แขน{ARM_TH[arm]}: ขยับ {len(mv)} ครั้ง · เหตุการณ์ "
              + (", ".join(f"{k} {n}" for k, n in kinds.items()) or "ไม่มี"))
        loads = [m["j1"]["peak"] for m in mv if m.get("j1")]
        if loads:
            over = sum(1 for v in loads if round(v) > 100)
            print(f"   ภาระ J1 สูงสุดต่อครั้ง: มากสุด {max(loads):.0f}% · "
                  f"เกิน 100% {over} ครั้ง ({100 * over / len(loads):.0f}%)")
        # รูด / แขนตก = เรื่องน้ำหนักที่ J1 (ชน = เรื่องทาง ไม่เอามาเทียบภาระ)
        slips = [r for r in mine if r["kind"] in LOAD_KINDS and r.get("move") and r["move"].get("j1")]
        if slips:
            print("   ตอนรูด / แขนตก:")
            for r in slips:
                m = r["move"]
                j = m["j1"]
                way = {True: "ยกขึ้น", False: "ลดลง"}.get(j.get("lift_at_peak"), "J1 แทบไม่หมุน")
                print(f"     {r['time'][5:16]} {r['kind']:<8} {arm_log.move_text(m)} — ภาระสูงสุด {j['peak']:.0f}% "
                      f"({way} {j.get('speed_at_peak', 0):.0f}°/s) ปลายทาง {j['to']:.0f}%")
            low = min(r["move"]["j1"]["peak"] for r in slips)
            risky = [m for m in mv if m.get("j1") and m["j1"]["peak"] >= low and (arm, m["ts"]) not in bad_ts]
            print(f"   ครั้งที่ภาระต่ำสุด {low:.0f}% — "
                  f"การขยับอื่นที่ภาระถึงระดับนี้แต่ยังไม่เป็นไร: {len(risky)} ครั้ง")
        groups = {}
        for m in mv:
            g = groups.setdefault(pose_key(m), {"n": 0, "bad": 0, "peak": 0.0, "to": 0.0})
            g["n"] += 1
            g["bad"] += (arm, m["ts"]) in bad_ts
            if m.get("j1"):
                g["peak"] = max(g["peak"], m["j1"]["peak"])
                g["to"] = max(g["to"], m["j1"]["to"])
        print("   ปลายทาง            ครั้ง  เกิดเรื่อง  ภาระ J1 สูงสุดระหว่างทาง  ที่ปลายทาง")
        for name, g in sorted(groups.items(), key=lambda kv: (-kv[1]["bad"], -kv[1]["peak"]))[:15]:
            flag = " ✗" if g["bad"] else (" ⚠" if round(g["to"]) > 100 else "")
            print(f"   {name:<18} {g['n']:5d}  {g['bad']:9d}  {g['peak']:20.0f}%  {g['to']:9.0f}%{flag}")
        advice(arm, mv, slips, groups)
        print()


def advice(arm, mv, slips, groups):
    tips = []
    for r in slips:  # ภาระสูงสุดอยู่ตรงไหน: ปลายทาง / ต้นทาง / ระหว่างทาง -> แก้คนละแบบ
        m, j = r["move"], r["move"]["j1"]
        if round(j["peak"]) <= 100:
            continue
        if j["to"] >= j["peak"] - 5:
            tip = (f"สอนท่า {m.get('to_pose') or 'ปลายทาง ' + arm_log.angles(m['to'])} ใหม่ให้แขนหุบเข้าหาแกน J1 "
                   f"(ค้างท่าภาระ {j['to']:.0f}%)")
        elif j["from"] >= j["peak"] - 5:
            tip = (f"ท่าเริ่ม {m.get('from_pose') or arm_log.angles(m['from'])} หนักอยู่แล้ว ({j['from']:.0f}%) — "
                   "สอนใหม่ให้หุบเข้า หรือพับแขนก่อนออกจากท่านี้")
        else:
            tip = (f"ทาง {arm_log.move_text(m)} ผ่านช่วงแขนยื่น (ภาระ {j['peak']:.0f}% ที่ J1 {j['peak_at'][0]:.0f}°) "
                   "ทั้งที่ต้นและปลายเบากว่า — แทรกท่าพับแขนระหว่างทาง แล้วค่อยหมุน J1")
        if tip not in tips:
            tips.append(tip)
    if tips:
        tips.append(f"ดูภาระทุกท่า: arm j1-load {arm}")
    heavy = [n for n, g in groups.items() if not g["bad"] and round(g["to"]) > 100 and not n.startswith("~")]
    if heavy:
        tips.append("ยังไม่เคยเกิดเรื่องแต่ค้างท่าหนักกว่าตอน calibrate: " + ", ".join(sorted(heavy)))
    if slips:
        lowering = sum(1 for r in slips if r["move"]["j1"].get("lift_at_peak") is False)
        if lowering > len(slips) / 2:
            tips.append("ส่วนใหญ่เกิดตอนแขนลดลง (มอเตอร์ต้องเบรกน้ำหนัก) — เช็กกระแสไดรเวอร์ J1 และลดความเร่ง")
        light = [r for r in slips if round(r["move"]["j1"]["peak"]) <= 100]
        if light:
            tips.append(f"{len(light)} ครั้งเกิดตอนภาระไม่เกิน 100% — น่าสงสัยสายพานหย่อน / ไดรเวอร์ "
                        "(SW4 ลดกระแสตอนหยุด) มากกว่าท่า")
    if not slips and not tips:
        tips.append("ยังไม่มีเหตุการณ์ให้เทียบ — เกิดเรื่องเมื่อไหร่พิมพ์ arm broke slipped ทันที")
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
