"""Arm motion log files (written by motion_log) and incident records (written by arm broke) — no ROS

Folder: env var ARM_LOG_DIR > /humanoi_ws/arm_logs (Pi: ~/Humanoi/ros2_ws/arm_logs) > ~/.ros/arm_logs
  moves-YYYY-MM-DD.jsonl   1 line = 1 arm move (start pose -> target pose, J1 load, speed, command source)
  incidents.jsonl          1 line = 1 incident logged by the user (slip/collision/...) with a copy of the related moves
Times in the files: ts = epoch seconds, time = Thai time (the container runs on UTC, so we add 7 h ourselves)
"""
import glob
import json
import math
import os
import time
from datetime import datetime, timedelta, timezone

LOCAL_TZ = timezone(timedelta(hours=7))  # Thai time
POSE_MATCH_DEG = 1.0  # every joint within this of a taught pose = counts as that pose


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
                        pass  # line left half-written by a power cut
    except FileNotFoundError:
        pass
    return out


def read_moves(since=None):
    """All moves (old -> new) · since = epoch seconds; reads only the files from that day on"""
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
    """Name of the taught pose matching these angles (every joint within POSE_MATCH_DEG) — no match = empty string"""
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
        return f"{s} s ago"
    if s < 3600:
        return f"{s // 60} min ago"
    if s < 86400:
        return f"{s // 3600} h ago"
    return f"{s // 86400} days ago"


def angles(vals):
    return "[" + " ".join(f"{v:.0f}" for v in vals) + "]"


def move_text(m):
    """One move as short text: start pose → target pose"""
    a = m.get("from_pose") or angles(m["from"])
    b = m.get("to_pose") or angles(m["to"])
    return f"{a} → {b}"


def j1_text(m):
    j = m.get("j1")
    if not j:
        return "J1 load ?"
    way = {True: " lifting", False: " lowering"}.get(j.get("lift_at_peak"), "")
    return f"J1 load max {j['peak']:.0f}%{way} (at target {j['to']:.0f}%)"
