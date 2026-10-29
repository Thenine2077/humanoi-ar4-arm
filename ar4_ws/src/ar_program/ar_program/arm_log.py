"""ไฟล์ log การขยับแขน (motion_log เขียน) และบันทึกเหตุการณ์ (arm broke เขียน) — ไม่ใช้ ROS

โฟลเดอร์: ตัวแปร ARM_LOG_DIR > /humanoi_ws/arm_logs (Pi: ~/Humanoi/ros2_ws/arm_logs) > ~/.ros/arm_logs
  moves-YYYY-MM-DD.jsonl   1 บรรทัด = แขนขยับ 1 ครั้ง (ท่าก่อน -> ท่าที่ไป ภาระ J1 ความเร็ว ที่มาของคำสั่ง)
  incidents.jsonl          1 บรรทัด = เหตุการณ์ที่ผู้ใช้จด (รูด/ชน/...) พร้อมสำเนาการขยับที่เกี่ยวข้อง
เวลาในไฟล์: ts = epoch วินาที, time = เวลาไทย (container ตั้งเป็น UTC จึงบวก 7 ชม. เอง)
"""
import glob
import json
import math
import os
import time
from datetime import datetime, timedelta, timezone

LOCAL_TZ = timezone(timedelta(hours=7))  # เวลาไทย
POSE_MATCH_DEG = 1.0  # ห่างท่าที่สอนไว้ไม่เกินนี้ทุกข้อ = ถือว่าเป็นท่านั้น


def log_dir():
    env = os.environ.get("ARM_LOG_DIR")
    if env:
        return env
    if os.path.isdir("/humanoi_ws"):
        return "/humanoi_ws/arm_logs"
    return os.path.expanduser("~/.ros/arm_logs")


def local_time(ts):
    return datetime.fromtimestamp(ts, LOCAL_TZ).strftime("%Y-%m-%d %H:%M:%S")


def moves_file(ts):
    return os.path.join(log_dir(), f"moves-{datetime.fromtimestamp(ts, LOCAL_TZ):%Y-%m-%d}.jsonl")


def incidents_file():
    return os.path.join(log_dir(), "incidents.jsonl")


def append(path, record):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def _read(path):
    out = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass  # บรรทัดที่เขียนค้างตอนไฟดับ
    except FileNotFoundError:
        pass
    return out


def read_moves(since=None):
    """การขยับทั้งหมด (เก่า -> ใหม่) · since = epoch วินาที อ่านเฉพาะไฟล์วันนั้นเป็นต้นไป"""
    files = sorted(glob.glob(os.path.join(log_dir(), "moves-*.jsonl")))
    if since is not None:
        first = os.path.basename(moves_file(since))
        files = [f for f in files if os.path.basename(f) >= first]
    moves = [r for f in files for r in _read(f) if r.get("type") == "move"]
    if since is not None:
        moves = [m for m in moves if m["ts"] >= since]
    return sorted(moves, key=lambda m: m["ts"])


def read_incidents():
    return [r for r in _read(incidents_file()) if r.get("type") == "incident"]


def pose_name(poses, values_rad):
    """ชื่อท่าที่สอนไว้ซึ่งตรงกับมุมนี้ (ทุกข้อห่างไม่เกิน POSE_MATCH_DEG) — ไม่ตรงท่าไหน = ข้อความว่าง"""
    best, best_d = "", POSE_MATCH_DEG
    for name, vals in (poses or {}).items():
        try:
            d = max(abs(math.degrees(a - b)) for a, b in zip(values_rad, vals))
        except TypeError:
            continue
        if d <= best_d:
            best, best_d = name, d
    return best


def ago(ts, now=None):
    s = int((now or time.time()) - ts)
    if s < 60:
        return f"{s} วิที่แล้ว"
    if s < 3600:
        return f"{s // 60} นาทีที่แล้ว"
    if s < 86400:
        return f"{s // 3600} ชม.ที่แล้ว"
    return f"{s // 86400} วันที่แล้ว"


def angles(vals):
    return "[" + " ".join(f"{v:.0f}" for v in vals) + "]"


def move_text(m):
    """การขยับ 1 ครั้งเป็นข้อความสั้น: ท่าก่อน → ท่าที่ไป"""
    a = m.get("from_pose") or angles(m["from"])
    b = m.get("to_pose") or angles(m["to"])
    return f"{a} → {b}"


def j1_text(m):
    j = m.get("j1")
    if not j:
        return "ภาระ J1 ?"
    way = {True: " ยกขึ้น", False: " ลดลง"}.get(j.get("lift_at_peak"), "")
    return f"ภาระ J1 สูงสุด {j['peak']:.0f}%{way} (ปลายทาง {j['to']:.0f}%)"
