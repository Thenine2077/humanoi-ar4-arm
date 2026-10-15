"""อ่าน/เขียน/ตรวจไฟล์ท่าและโปรแกรมของแขน AR4 (ใช้ร่วมกันระหว่าง teach กับ arm_program)

รูปแบบไฟล์ (YAML):

  poses:                       # มุมข้อต่อ joint_1..joint_6 หน่วยเรเดียน (ลำดับเดียวกับ JOINTS)
    rest: [0.0, -0.1019, 0.0, 0.0, -0.9103, 0.0]
  programs:                    # ลำดับขั้น: ชื่อท่า หรือ {wait: วินาที}
    demo: [rest, look_left, {wait: 1.0}, rest]

ไฟล์นี้เป็นข้อมูลของผู้ใช้ — ค่าเริ่มต้นจึงชี้ไปที่ src/ ไม่ใช่ install/ (install ถูกเขียนทับทุกครั้ง
ที่ colcon build ท่าที่ teach ไว้จะหายหมด)
"""
import math
import os

import yaml

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# ขอบเขตข้อต่อ (rad) — ค่าเดียวกับ <ros2_control> ใน ar_gz_bringup/urdf/ar_gz.urdf.xacro ซึ่งแปลง
# มาจาก ar.ros2_control.xacro ของผู้ขาย ตรวจทุกท่าก่อนสั่งแขน กันไฟล์ที่พิมพ์ผิดสั่งชนลิมิต
LIMITS = {
    "joint_1": (-2.967060, 2.967060),
    "joint_2": (-0.733038, 1.570796),
    "joint_3": (-1.553343, 0.907571),
    "joint_4": (-2.879793, 2.879793),
    "joint_5": (-1.832596, 1.832596),
    "joint_6": (-2.705260, 2.705260),
}

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
# ท่าและโปรแกรมของแขน AR4 — แก้ด้วยมือได้ หรือบันทึกท่าด้วย:
#   ros2 run ar_program teach <ชื่อท่า>
#   ros2 run ar_program teach --program <ชื่อโปรแกรม> <ท่า1> <ท่า2> wait=1.0 <ท่า3> ...
# สั่งรัน:  ros2 topic pub --once /arm/run std_msgs/msg/String "{data: <ชื่อโปรแกรม>}"
#
# poses    : มุม joint_1..joint_6 หน่วยเรเดียน
# programs : ลำดับขั้น — ชื่อท่า หรือ {wait: วินาที}
# (ไฟล์นี้ถูกเขียนใหม่ทั้งไฟล์ทุกครั้งที่ teach คอมเมนต์ที่เพิ่มเองจะหาย ยกเว้นส่วนหัวนี้)

"""


class ProgramError(ValueError):
    pass


def load(path):
    if not os.path.exists(path):
        return {"poses": {}, "programs": {}}
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    data.setdefault("poses", {})
    data.setdefault("programs", {})
    if data["poses"] is None:
        data["poses"] = {}
    if data["programs"] is None:
        data["programs"] = {}
    return data


def save(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    poses = {k: [round(float(v), 4) for v in vals] for k, vals in data["poses"].items()}
    body = yaml.safe_dump(
        {"poses": poses, "programs": data["programs"]},
        allow_unicode=True,
        default_flow_style=None,  # list ของตัวเลขจะเป็นบรรทัดเดียว อ่านง่าย
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


def trapezoid(start, target, speed, accel, min_time=0.0, dt=0.1):
    """แผนเคลื่อนที่แบบ เร่ง → วิ่งความเร็วคงที่ → เบรก ทุกข้อต่อเริ่มและจบพร้อมกัน

    ข้อต่อที่ต้องไปไกลที่สุดวิ่งด้วย speed (rad/s) เร่ง/เบรกด้วย accel (rad/s^2) ข้อต่ออื่นย่อสัดส่วนตาม
    คืน [(t, positions, velocities), ...] เรียงตามเวลา จุดสุดท้าย = target ความเร็ว 0

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
    stretch = max(1.0, min_time / t_total)  # ท่าที่ใกล้กันมาก ยืดเวลาให้ถึง min_time กันกระชาก

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


def resolve(data, program):
    """แปลงโปรแกรมเป็นรายการขั้นที่ตรวจแล้ว: [("pose", name, [6 ค่า]) | ("wait", None, วินาที)]

    ตรวจทั้งโปรแกรมก่อนเริ่มขยับ — ถ้ามีท่าที่หาไม่เจอหรือเกินลิมิตแม้แต่ขั้นเดียว
    จะไม่ขยับเลย ดีกว่าวิ่งไปครึ่งทางแล้วค่อยพัง
    """
    if program.startswith("@"):  # "@ชื่อท่า" = ไปท่าเดียว (ใช้ตอนแก้ท่า: ไปท่าเดิม -> ปรับ -> teach ทับ)
        pose = program[1:]
        if pose not in data["poses"]:
            known = ", ".join(sorted(data["poses"])) or "(ยังไม่มี)"
            raise ProgramError(f"ไม่มีท่า '{pose}' — ที่มี: {known}")
        return [("pose", pose, check_pose(pose, data["poses"][pose]))]
    steps = data["programs"].get(program)
    if steps is None:
        known = ", ".join(sorted(data["programs"])) or "(ยังไม่มี)"
        raise ProgramError(f"ไม่มีโปรแกรม '{program}' — ที่มี: {known}")
    if not isinstance(steps, list) or not steps:
        raise ProgramError(f"โปรแกรม '{program}' ต้องเป็นรายการขั้นที่ไม่ว่าง")
    out = []
    for i, step in enumerate(steps, 1):
        if isinstance(step, str):
            if step not in data["poses"]:
                raise ProgramError(f"โปรแกรม '{program}' ขั้นที่ {i}: ไม่มีท่า '{step}'")
            out.append(("pose", step, check_pose(step, data["poses"][step])))
        elif isinstance(step, dict) and set(step) == {"wait"}:
            secs = float(step["wait"])
            if not 0 <= secs <= 600:
                raise ProgramError(f"โปรแกรม '{program}' ขั้นที่ {i}: wait ต้องอยู่ระหว่าง 0-600 วินาที")
            out.append(("wait", None, secs))
        else:
            raise ProgramError(
                f"โปรแกรม '{program}' ขั้นที่ {i}: ไม่รู้จัก {step!r} (ใช้ชื่อท่า หรือ {{wait: วินาที}})"
            )
    return out
