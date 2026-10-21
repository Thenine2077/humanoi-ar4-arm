"""อ่าน/เขียน/ตรวจไฟล์ท่าและโปรแกรมของแขน AR4 สองตัว (ใช้ร่วมกันระหว่าง teach กับ arm_program)

แขน 1 = left (แขนซ้ายเมื่อมองจากตัวหุ่น), แขน 2 = right

รูปแบบไฟล์ (YAML):

  poses:                        # แยกตามแขน — ชื่อท่าซ้ำกันได้ข้ามแขน (มุมเป็นของแต่ละแขน)
    left:
      rest: [0.0, -0.1019, 0.0, 0.0, -0.9103, 0.0]   # joint_1..joint_6 เรเดียน
    right:
      rest: [0.0, -0.1019, 0.0, 0.0, -0.9103, 0.0]
  programs:                     # ทีละขั้น — แต่ละขั้นบอกท่าของแต่ละแขน หรือ still (ไม่สั่งแขนนั้น = ค้างอยู่ที่เดิม)
    pick2:
      - {left: act1, right: act1}
      - {left: act2, right: still, left_hand: rock}   # ท่ามือ (ไม่ใส่ = มือไม่ขยับ) — ขยับพร้อมแขนในขั้นเดียวกัน
      - {left: still, right: still, left_hand: paper} # มืออย่างเดียว
      - {wait: 1.0}
    demo: [rest, look_left]     # แบบเก่า (ชื่อท่าเปล่าๆ) = แขนซ้ายอย่างเดียว

  hand_poses:                   # ท่ามือที่สอนเอง (arm hand-teach): องศา servo ID 1-8 ตรงๆ ของชุดมือขวา
    wave: [10, 40, -20, 30, 90, -90, 0, 0]       # ชื่อเดียวกับท่าในตัว (rock paper ...) = ใช้แทนท่าในตัว
  hand_poses_left: {}           # ของชุดมือซ้าย (มือคนละตัว ประกอบกลับด้าน — องศาใช้แทนกันไม่ได้)

  still ไม่ใช่ท่าที่จำตำแหน่งไว้ — แขนจะค้างที่ที่ขั้นก่อนหน้าพาไป ถ้าจำตำแหน่งตอนเขียนโปรแกรม แขนจะวิ่งกลับไปที่นั่น
  ทุกขั้น แขนที่ต้องขยับเริ่มพร้อมกันและถึงพร้อมกัน ขั้นถัดไปเริ่มเมื่อทุกแขนถึงแล้ว
  ไฟล์แบบเก่า (poses เป็นท่าแขนเดียว) อ่านได้ — ถือเป็นท่าแขนซ้าย แล้วเขียนกลับเป็นแบบใหม่ตอนบันทึกครั้งถัดไป

ไฟล์นี้เป็นข้อมูลของผู้ใช้ — ค่าเริ่มต้นจึงชี้ไปที่ src/ ไม่ใช่ install/ (install ถูกเขียนทับทุกครั้ง
ที่ colcon build ท่าที่ teach ไว้จะหายหมด)
"""
import math
import os

import yaml

ARMS = ("left", "right")
ARM_ALIASES = {"left": "left", "l": "left", "1": "left", "ซ้าย": "left",
               "right": "right", "r": "right", "2": "right", "ขวา": "right"}
STILL = ("still", "-")
HAND_KEYS = {"left": "left_hand", "right": "right_hand"}  # มือบนแขนซ้าย / แขนขวา
# ท่ามือที่สอนเอง แยกตามชุดมือ (ชุดมือขวา / ชุดมือซ้าย ของ AmazingHand — ไม่เกี่ยวกับว่าติดแขนไหน)
HAND_POSE_KEYS = {"right": "hand_poses", "left": "hand_poses_left"}
# ท่ามือที่สั่งได้ -> เวลาที่มือใช้ (วินาที) ขั้นถัดไปรอจนครบ — ชื่อและท่าจริงอยู่ใน humanoi_games/hand_poses.py
# (Humanoi repo) แก้คู่กัน · victory / nonono = ท่าเคลื่อนไหว
HAND_GESTURES = {"rock": 0.6, "paper": 0.6, "scissors": 0.6, "point": 0.6, "like": 0.6, "windup": 0.6,
                 "fist": 0.6, "victory": 1.8, "nonono": 1.8, "relax": 0.3}
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# ขอบเขตข้อต่อ (rad) — ค่าเดียวกับ <ros2_control> ใน ar_dual/urdf/humanoi_arms.urdf.xacro ซึ่งแปลงมาจาก
# ar.ros2_control.xacro ของผู้ขาย ตรวจทุกท่าก่อนสั่งแขน กันไฟล์ที่พิมพ์ผิดสั่งชนลิมิต (สองแขนเหมือนกัน)
LIMITS = {
    "joint_1": (-2.967060, 2.967060),
    "joint_2": (-0.733038, 1.570796),
    "joint_3": (-1.553343, 0.907571),
    "joint_4": (-2.879793, 2.879793),
    "joint_5": (-1.832596, 1.832596),
    "joint_6": (-2.705260, 2.705260),
}


def joint_names(arm, layout="dual"):
    """ชื่อข้อต่อใน /joint_states: dual = left_joint_1.. / right_joint_1.., single (แขนเดี่ยวแบบเดิม) = joint_1.."""
    return [f"{arm}_{j}" for j in JOINTS] if layout == "dual" else list(JOINTS)


def arm_of(token):
    arm = ARM_ALIASES.get(str(token).strip().lower())
    if arm is None:
        raise ProgramError(f"ไม่รู้จักแขน '{token}' — ใช้ left/1 หรือ right/2")
    return arm


def _default_file():
    """หาไฟล์ท่าใน src/ ของเครื่องที่รันอยู่ (ต้องเป็น src ไม่ใช่ install — install ถูกเขียนทับตอน build)

    ลำดับ: ตัวแปร AR_PROGRAM_FILE > PC (WSL) > Pi (Docker humanoi_ros, ros2_ws mount ที่ /humanoi_ws)
    """
    env = os.environ.get("AR_PROGRAM_FILE")
    if env:
        return env
    for path in (
        "/mnt/c/Users/User/Desktop/ar4/ar4_ws/src/ar_program/config/programs.yaml",  # PC / WSL
        "/humanoi_ws/src/ar_program/config/programs.yaml",                          # Pi / Docker
    ):
        if os.path.isdir(os.path.dirname(path)):
            return path
    return os.path.expanduser("~/ar_program/programs.yaml")


DEFAULT_FILE = _default_file()

HEADER = """\
# ท่าและโปรแกรมของแขน AR4 สองตัว (left = แขน 1 ซ้ายเมื่อมองจากตัวหุ่น, right = แขน 2)
# แก้ด้วยมือได้ หรือใช้คำสั่งบน Pi:
#   arm teach <left|right> <ชื่อท่า>                        บันทึกท่าปัจจุบันของแขนนั้น
#   arm program <ชื่อ> <ซ้าย>/<ขวา> ... wait=1.0 ...         เช่น  arm program pick2 act1/act1 act2/- -/act3
#   arm show <ชื่อโปรแกรม>                                 ดูเป็นตาราง
#
# poses    : แยกตามแขน มุม joint_1..joint_6 หน่วยเรเดียน
# programs : ทีละขั้น — {left: ท่า, right: ท่า|still, left_hand: ท่ามือ} หรือ {wait: วินาที}
#            (ชื่อท่าเปล่าๆ = แขนซ้ายอย่างเดียว)
# hand_poses: ท่ามือที่สอนเอง (arm hand-teach) องศา servo ID 1-8 — ชื่อซ้ำท่าในตัว = ใช้แทนท่าในตัว
#             hand_poses = ชุดมือขวา, hand_poses_left = ชุดมือซ้าย (arm hand-side บอกว่าตอนนี้ใส่ตัวไหน)
# (ไฟล์นี้ถูกเขียนใหม่ทั้งไฟล์ทุกครั้งที่ teach คอมเมนต์ที่เพิ่มเองจะหาย ยกเว้นส่วนหัวนี้)

"""


class ProgramError(ValueError):
    pass


def load(path):
    data = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    poses = data.get("poses") or {}
    # แบบเก่า: {ชื่อท่า: [6 ค่า]} = ท่าของแขนเดียว -> แขนซ้าย
    if poses and all(isinstance(v, (list, tuple)) for v in poses.values()):
        poses = {"left": poses}
    data["poses"] = {arm: dict(poses.get(arm) or {}) for arm in ARMS}
    data["programs"] = data.get("programs") or {}
    for key in HAND_POSE_KEYS.values():
        data[key] = {str(k): [float(v) for v in vals] for k, vals in (data.get(key) or {}).items()}
    return data


def save(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    poses = {arm: {k: [round(float(v), 4) for v in vals] for k, vals in data["poses"][arm].items()}
             for arm in ARMS}
    out = {"poses": poses}
    for key in HAND_POSE_KEYS.values():
        hand_poses = {k: [round(float(v), 1) for v in vals] for k, vals in (data.get(key) or {}).items()}
        if hand_poses or key == "hand_poses":
            out[key] = hand_poses
    out["programs"] = data["programs"]
    body = yaml.safe_dump(
        out,
        allow_unicode=True,
        default_flow_style=None,  # list ของตัวเลข / dict ของขั้น จะเป็นบรรทัดเดียว อ่านง่าย
        sort_keys=False,
    )
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(HEADER + body)
    os.replace(tmp, path)  # เขียนไฟล์ชั่วคราวก่อนแล้วสลับ กันไฟล์พังถ้าโปรแกรมหยุดกลางทาง


def check_pose(name, values):
    """คืน list มุม 6 ค่าถ้าใช้ได้ ไม่งั้น raise ProgramError พร้อมเหตุผล"""
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINTS):
        raise ProgramError(f"ท่า '{name}' ต้องมี {len(JOINTS)} ค่า (joint_1..joint_6)")
    out = []
    for joint, v in zip(JOINTS, values):
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise ProgramError(f"ท่า '{name}' {joint} ไม่ใช่ตัวเลข: {v!r}")
        lo, hi = LIMITS[joint]
        if not lo <= v <= hi:
            raise ProgramError(
                f"ท่า '{name}' {joint} = {v:.3f} rad ({math.degrees(v):.1f}°) "
                f"เกินขอบเขต {lo:.3f}..{hi:.3f} rad"
            )
        out.append(v)
    return out


def step_arms(step):
    """ขั้นในไฟล์ -> {แขน: ชื่อท่า|still} (ชื่อท่าเปล่าๆ แบบเก่า = แขนซ้าย) หรือ None ถ้าเป็น wait"""
    if isinstance(step, str):
        return {"left": step, "right": "still"}
    if isinstance(step, dict) and set(step) == {"wait"}:
        return None
    if isinstance(step, dict) and step and set(step) <= set(ARMS) | set(HAND_KEYS.values()):
        return {arm: str(step.get(arm, "still")) for arm in ARMS}
    raise ProgramError(f"ไม่รู้จักขั้น {step!r} (ใช้ {{left: ท่า, right: ท่า|still, left_hand: ท่ามือ}} "
                       "หรือ {wait: วินาที})")


def hand_names(data=None):
    """ชื่อท่ามือที่สั่งได้: ท่าในตัว + ท่าที่สอนเองในไฟล์ (มือทั้งสองชุด)"""
    names = set(HAND_GESTURES)
    for key in HAND_POSE_KEYS.values():
        names |= set((data or {}).get(key) or {})
    return names


def hand_duration(name):
    """เวลาที่มือใช้ทำท่า (วินาที) — ท่าที่สอนเองเป็นท่านิ่ง"""
    return HAND_GESTURES.get(name, 0.6)


def step_hands(step):
    """ขั้นในไฟล์ -> {แขน: ท่ามือ} เฉพาะมือที่สั่ง (ไม่มี = มือไม่ขยับ)"""
    if not isinstance(step, dict):
        return {}
    return {arm: str(step[key]) for arm, key in HAND_KEYS.items() if str(step.get(key, "still")) not in STILL}


def parse_cli_step(token, hands=None):
    """ข้อความหนึ่งขั้นจากคำสั่ง -> ขั้นในไฟล์ (hands = ชื่อท่ามือที่รับได้ ไม่ใส่ = ท่าในตัว)

    "ท่าซ้าย/ท่าขวา[/มือซ้าย[/มือขวา]]" (- หรือ still = ไม่สั่ง) หรือ "wait=วินาที"
    """
    if token.startswith("wait="):
        return {"wait": float(token[5:])}
    parts = [p.strip() for p in token.split("/")]
    if not 2 <= len(parts) <= 4 or not all(parts):
        raise ProgramError(f"'{token}' — แต่ละขั้นเขียน <แขนซ้าย>/<แขนขวา>[/<มือซ้าย>[/<มือขวา>]] "
                           "(ใช้ - ถ้าไม่ขยับ) หรือ wait=วินาที")
    step = {"left": "still" if parts[0] in STILL else parts[0], "right": "still" if parts[1] in STILL else parts[1]}
    for arm, part in zip(ARMS, parts[2:]):
        if part in STILL:
            continue
        known = hands if hands is not None else HAND_GESTURES
        if part not in known:
            raise ProgramError(f"'{token}' — ไม่รู้จักท่ามือ '{part}' (มี: {', '.join(sorted(known))})")
        step[HAND_KEYS[arm]] = part
    return step


def trapezoid(start, target, speed, accel, min_time=0.0, dt=0.1):
    """แผนเคลื่อนที่แบบ เร่ง → วิ่งความเร็วคงที่ → เบรก ทุกข้อต่อเริ่มและจบพร้อมกัน

    ข้อต่อที่ต้องไปไกลที่สุดวิ่งด้วย speed (rad/s) เร่ง/เบรกด้วย accel (rad/s^2) ข้อต่ออื่นย่อสัดส่วนตาม
    คืน [(t, positions, velocities), ...] เรียงตามเวลา จุดสุดท้าย = target ความเร็ว 0
    min_time ยืดเวลาให้ยาวขึ้น (ใช้ทั้งกันกระชากตอนท่าใกล้กัน และให้สองแขนในขั้นเดียวกันถึงพร้อมกัน)

    ทำไมไม่ส่งจุดเดียวแล้วให้ controller ลากเส้นโค้งเอง: เส้นโค้งแบบนั้นมีความเร็วยอด 1.5 เท่าของ
    ความเร็วเฉลี่ย ส่วนเฟิร์มแวร์ Teensy (AccelStepper เร่งได้ 10°/s^2) ตามหลังเป้าหมายราว v^2/2a
    ท่าไกลๆ จะตามหลังเกิน path tolerance 0.2 rad ของ controller แล้วโดน abort กลางทาง
    (เจอจริงกับท่า home 157°: ความเร็วยอด 17°/s → ตามหลัง ~15°)
    """
    deltas = [t - s for s, t in zip(start, target)]
    dist = max(abs(d) for d in deltas)
    if dist < 1e-6:
        return [(max(min_time, dt), list(target), [0.0] * len(target))]
    v, a = speed, accel
    if dist >= v * v / a:            # ไกลพอจะมีช่วงวิ่งความเร็วคงที่
        t_acc = v / a
        t_total = 2 * t_acc + (dist - v * v / a) / v
    else:                            # ใกล้ — เร่งยังไม่ถึง speed ก็ต้องเบรกแล้ว
        t_acc = math.sqrt(dist / a)
        v = a * t_acc
        t_total = 2 * t_acc
    stretch = max(1.0, min_time / t_total)  # ยืดเวลาให้ถึง min_time

    def at(t):  # ระยะที่ไปได้ (0..dist) และความเร็ว ณ เวลา t ของแผนก่อนยืด
        if t < t_acc:
            return 0.5 * a * t * t, a * t
        if t < t_total - t_acc:
            return 0.5 * a * t_acc * t_acc + v * (t - t_acc), v
        r = t_total - t
        return dist - 0.5 * a * r * r, a * r

    end = t_total * stretch
    points = []
    for k in range(1, int(math.ceil(end / dt)) + 1):
        t = min(k * dt, end)
        s, sv = at(t / stretch)
        frac, vfrac = s / dist, sv / stretch / dist
        points.append((t, [st + d * frac for st, d in zip(start, deltas)],
                       [d * vfrac for d in deltas]))
    points[-1] = (end, list(target), [0.0] * len(target))
    return points


def _pose(data, arm, name, where):
    if name not in data["poses"][arm]:
        known = ", ".join(sorted(data["poses"][arm])) or "(ยังไม่มี)"
        raise ProgramError(f"{where}: แขน{'ซ้าย' if arm == 'left' else 'ขวา'}ไม่มีท่า '{name}' — ที่มี: {known}")
    return check_pose(f"{arm}:{name}", data["poses"][arm][name])


def resolve(data, program, arms=ARMS):
    """แปลงโปรแกรมเป็นรายการขั้นที่ตรวจแล้ว: [("move", {แขน: (ชื่อท่า, [6 ค่า])}, {แขน: ท่ามือ}) | ("wait", วินาที, {})]

    arms = แขนที่มีอยู่จริงในระบบที่รัน (ระบบแขนเดี่ยวแบบเดิม = ("left",))
    ตรวจทั้งโปรแกรมก่อนเริ่มขยับ — ถ้ามีท่าที่หาไม่เจอหรือเกินลิมิตแม้แต่ขั้นเดียว จะไม่ขยับเลย
    ดีกว่าวิ่งไปครึ่งทางแล้วค่อยพัง
    """
    if program.startswith("@"):  # "@ท่า" (แขนซ้าย) หรือ "@right:ท่า" = ไปท่าเดียว (ใช้ตอนแก้ท่า)
        arm, _, pose = program[1:].rpartition(":")
        arm = arm_of(arm) if arm else "left"
        if arm not in arms:
            raise ProgramError(f"ระบบนี้ไม่มีแขน{arm}")
        return [("move", {arm: (pose, _pose(data, arm, pose, "goto"))}, {})]
    steps = data["programs"].get(program)
    if steps is None:
        known = ", ".join(sorted(data["programs"])) or "(ยังไม่มี)"
        raise ProgramError(f"ไม่มีโปรแกรม '{program}' — ที่มี: {known}")
    if not isinstance(steps, list) or not steps:
        raise ProgramError(f"โปรแกรม '{program}' ต้องเป็นรายการขั้นที่ไม่ว่าง")
    out = []
    for i, step in enumerate(steps, 1):
        where = f"โปรแกรม '{program}' ขั้นที่ {i}"
        try:
            per_arm = step_arms(step)
        except ProgramError as exc:
            raise ProgramError(f"{where}: {exc}")
        if per_arm is None:
            secs = float(step["wait"])
            if not 0 <= secs <= 600:
                raise ProgramError(f"{where}: wait ต้องอยู่ระหว่าง 0-600 วินาที")
            out.append(("wait", secs, {}))
            continue
        hands = step_hands(step)
        for arm, gesture in hands.items():
            if gesture not in hand_names(data):
                raise ProgramError(f"{where}: ไม่รู้จักท่ามือ '{gesture}' (มี: {', '.join(sorted(hand_names(data)))})")
        moves = {}
        for arm, pose in per_arm.items():
            if pose in STILL:
                continue
            if arm not in arms:
                raise ProgramError(f"{where}: ระบบที่รันอยู่ไม่มีแขน{'ขวา' if arm == 'right' else 'ซ้าย'}")
            moves[arm] = (pose, _pose(data, arm, pose, where))
        if not moves and not hands:
            raise ProgramError(f"{where}: ทุกแขนและมือ still — ถ้าจะรอให้ใช้ wait=วินาที")
        out.append(("move", moves, hands))
    return out


def table(data, program):
    """แถวสำหรับแสดงเป็นตาราง: [(ลำดับ, แขนซ้าย, แขนขวา, มือซ้าย, มือขวา)] — ใช้ทั้ง arm show และ GUI ในอนาคต"""
    rows = []
    for i, step in enumerate(data["programs"].get(program) or [], 1):
        per_arm = step_arms(step)
        if per_arm is None:
            rows.append((i, f"wait {float(step['wait']):g} s", "", "", ""))
        else:
            hands = step_hands(step)
            rows.append((i, per_arm["left"], per_arm["right"], hands.get("left", "-"), hands.get("right", "-")))
    return rows


def cli_step(row):
    """แถวของ table กลับเป็นข้อความแบบที่พิมพ์ในคำสั่ง เช่น act1/-/rock (ตัดช่องท้ายที่เป็น - ทิ้ง)"""
    _, left, right, lh, rh = row
    if not right and not lh:
        return left.replace("wait ", "wait=").replace(" s", "")
    parts = ["-" if p in STILL else p for p in (left, right, lh, rh)]
    while len(parts) > 2 and parts[-1] == "-":
        parts.pop()
    return "/".join(parts)
