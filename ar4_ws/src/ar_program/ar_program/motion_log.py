#!/usr/bin/env python3
"""motion_log — จดทุกครั้งที่แขนขยับ: ท่าก่อน -> ท่าที่ไป, ภาระ J1 ระหว่างทาง, ความเร็ว, ใครสั่ง, ผลของตัวคุม

เปิดเองพร้อมระบบแขน (ar_dual dual_hardware.launch.py = arm start) ไม่ต้องสั่งเอง
ไว้คู่กับ arm broke: เกิดเรื่อง (สายพานรูด แขนชน ...) พิมพ์ arm broke slipped แล้ว broke.py ดึงการขยับล่าสุด
จากไฟล์นี้ไปเก็บกับเหตุการณ์ — พอสะสมไว้หลายครั้งก็ดูได้ว่าท่า/ทางไหนรูดบ่อย (arm broke report)

แบ่งการขยับ 1 ครั้งตาม goal ของตัวคุมแขน (ทุกคำสั่งเข้าทางนี้: arm player / goto / MoveIt / เกม / หยิบกล่อง)
— goal ใหม่มา = ปิดครั้งก่อน เปิดครั้งใหม่ · ไม่มี goal (เช่นซิม) = ขยับเกิน 0.2° เริ่ม นิ่ง 0.4 วินาทีจบ
ตำแหน่งที่จดคือตัวเลขที่บอร์ดนับ (มอเตอร์ไม่มี encoder) — ตอนรูด ตัวเลขยังเป็นท่าที่สั่ง ไม่ใช่ท่าที่แขนอยู่จริง

หัวข้อ: /joint_states, /arm/state (ตัวเล่นโปรแกรม), <แขน>_arm_controller/follow_joint_trajectory/_action/status,
       /move_action และ /execute_trajectory/_action/status (MoveIt), /robot_description (คำนวณภาระ J1)
ไฟล์: ar_program/arm_log.py (<ARM_LOG_DIR>/moves-YYYY-MM-DD.jsonl)
"""
import math
import socket
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ar_program import arm_log, j1_load, programs

START_DEG = 0.2      # ขยับจากท่านิ่งเกินนี้ = เริ่มขยับ (ไม่มี goal)
STILL_S = 0.4        # ไม่ขยับนานเท่านี้ = จบ (ถ้าตัวคุมบอกผลแล้ว หรือไม่มี goal)
STILL_MAX_S = 3.0    # goal ยังไม่จบแต่แขนนิ่งนานขนาดนี้ = จบไปก่อน
SAMPLE_S = 0.1       # เก็บท่าระหว่างทางทุก 0.1 วินาที (ไว้คำนวณภาระ J1 / ความเร็ว)
MAX_SAMPLES = 600    # ยาวกว่านี้ (60 วินาที) ข้ามบางจุด
CHANGE_RAD = 1e-4    # ตัวเลขเปลี่ยนเกินนี้ = ยังขยับอยู่ (บอร์ดนับละเอียด ~0.02°)
# เปลี่ยนเกิน JUMP_DEG ในข้อความเดียว และเร็วเกิน JUMP_DEG_S = ตัวเลขกระโดด (เช่นหลัง recal ตัวคุมเพิ่งกลับมา)
# ไม่ใช่การขยับ — มอเตอร์หมุนได้ไม่เกิน ~31°/s · ข้อความมาช้า (เครื่องหน่วง) ยังนับเป็นการขยับ
JUMP_DEG = 3.0
JUMP_DEG_S = 120.0
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
RESULT = {GoalStatus.STATUS_SUCCEEDED: "ok", GoalStatus.STATUS_CANCELED: "canceled",
          GoalStatus.STATUS_ABORTED: "aborted"}


class Segment:
    def __init__(self, t, q, goal=None, src=""):
        self.t_open = t
        self.t0 = t
        self.q0 = list(q)
        self.goal = goal
        self.result = ""
        self.src = src
        self.moved = False
        self.samples = [(t, list(q))]
        self.first_change = None
        self.last_change = t


class MotionLog(Node):
    def __init__(self):
        super().__init__("arm_motion_log")
        self.arms = {}  # แขน -> {"anchor": ท่านิ่งล่าสุด, "seg": Segment|None, "last": ท่าล่าสุด}
        self.goals = {arm: {} for arm in programs.ARMS}  # uuid -> สถานะล่าสุด
        self.moveit_active = 0.0
        self.arm_state = ""
        self.loader = None
        self.poses_cache = (None, {})
        self.t_start = time.time()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 20)
        self.create_subscription(String, "/arm/state", self.on_arm_state, 10)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        for arm in programs.ARMS:
            self.create_subscription(GoalStatusArray, f"/{arm}_arm_controller/follow_joint_trajectory/_action/status",
                                     lambda m, a=arm: self.on_goals(a, m), 10)
        for name in ("/move_action", "/execute_trajectory"):
            self.create_subscription(GoalStatusArray, f"{name}/_action/status", self.on_moveit, 10)
        self.create_timer(3.0, self.fallback_description)
        self.get_logger().info(f"บันทึกการขยับแขนลง {arm_log.log_dir()} (arm broke = จดเหตุการณ์)")

    # ---------- โมเดล (ภาระ J1) ----------
    def on_description(self, msg):
        if self.loader is None:
            self.set_loader(msg.data, "/robot_description")

    def fallback_description(self):
        if self.loader is None and time.time() - self.t_start > 5.0:
            try:
                self.set_loader(j1_load.urdf_text(), "xacro")
            except Exception as exc:  # ไม่มีโมเดล = จดได้ แต่ไม่มีภาระ J1
                self.get_logger().warn(f"โหลดโมเดลไม่ได้ ({exc}) — ไม่คำนวณภาระ J1")
                self.loader = False

    def set_loader(self, xml, where):
        try:
            loader = j1_load.J1Load(xml)
            ref = {arm: round(loader.reference(arm), 3) for arm in programs.ARMS if loader.has_arm(arm)}
        except Exception as exc:
            self.get_logger().warn(f"โมเดลจาก {where} ใช้คำนวณภาระ J1 ไม่ได้: {exc}")
            return
        self.loader = loader
        now = time.time()
        arm_log.append(arm_log.moves_file(now), {
            "type": "session", "ts": round(now, 3), "time": arm_log.local_time(now), "host": socket.gethostname(),
            "model": where, "j1_ref_nm": ref, "speed_cap": programs_speed_note()})

    # ---------- ที่มาของคำสั่ง ----------
    def on_arm_state(self, msg):
        self.arm_state = msg.data

    def on_moveit(self, msg):
        if any(s.status in ACTIVE for s in msg.status_list):
            self.moveit_active = time.time()

    def source(self):
        if self.arm_state.startswith("running:"):
            return "program " + self.arm_state[len("running:"):]
        if time.time() - self.moveit_active < 1.0:
            return "moveit"
        return "other"

    def on_goals(self, arm, msg):
        known = self.goals[arm]
        now = time.time()
        for s in msg.status_list:
            uid = bytes(s.goal_info.goal_id.uuid)
            old = known.get(uid)
            known[uid] = s.status
            if old is None and s.status in ACTIVE:
                self.new_goal(arm, uid, now)
            if s.status in RESULT and old != s.status:
                st = self.arms.get(arm)
                if st and st["seg"] is not None and st["seg"].goal == uid:
                    st["seg"].result = RESULT[s.status]
        if len(known) > 200:  # goal เก่าสะสม — เก็บไว้แค่ที่ยังอยู่ในข้อความล่าสุด
            keep = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list}
            self.goals[arm] = {k: v for k, v in known.items() if k in keep}

    def new_goal(self, arm, uid, now):
        st = self.arms.get(arm)
        if st is None:
            return
        seg = st["seg"]
        if seg is not None and seg.moved and seg.goal is None and now - seg.t0 < 1.0:
            seg.goal, seg.src = uid, self.source()  # สถานะ goal มาช้ากว่าแขนเริ่มขยับนิดหน่อย = ครั้งเดียวกัน
            return
        if seg is not None and seg.moved:
            self.close(arm, now, "goal ใหม่")
            seg = None
        if seg is None:
            st["seg"] = Segment(now, st["last"], uid, self.source())
        else:
            seg.goal, seg.src = uid, self.source()

    # ---------- ท่าแขน ----------
    def on_joints(self, msg):
        now = time.time()
        pos = dict(zip(msg.name, msg.position))
        for arm in programs.ARMS:
            names = programs.joint_names(arm)
            if not all(n in pos for n in names):
                continue
            q = [pos[n] for n in names]
            st = self.arms.get(arm)
            if st is None:
                self.arms[arm] = {"anchor": q, "seg": None, "last": q, "t_last": now}
                continue
            self.step(arm, st, q, now)
            st["last"], st["t_last"] = q, now

    def step(self, arm, st, q, now):
        jump = max(abs(a - b) for a, b in zip(q, st["last"]))
        dt = max(now - st["t_last"], 1e-3)
        if jump > math.radians(JUMP_DEG) and math.degrees(jump) / dt > JUMP_DEG_S:
            # ตัวคุมแขนเพิ่งกลับมา (recal / reconnect) ตัวเลขเปลี่ยนทันที — จดไว้เฉยๆ ไม่นับเป็นการขยับ
            arm_log.append(arm_log.moves_file(now), {
                "type": "jump", "ts": round(now, 3), "time": arm_log.local_time(now), "arm": arm,
                "from": [round(math.degrees(v), 2) for v in st["last"]], "to": [round(math.degrees(v), 2) for v in q]})
            st["anchor"], st["seg"] = q, None
            return
        seg = st["seg"]
        if seg is None:
            if max(abs(a - b) for a, b in zip(q, st["anchor"])) > math.radians(START_DEG):
                seg = st["seg"] = Segment(now, st["anchor"], None, self.source())
            else:
                return
        if jump > CHANGE_RAD:
            seg.last_change = now
            if seg.first_change is None:
                seg.first_change = now
        if not seg.moved and max(abs(a - b) for a, b in zip(q, seg.q0)) > math.radians(START_DEG):
            seg.moved = True
            seg.t0 = seg.first_change or now
        if now - seg.samples[-1][0] >= SAMPLE_S:
            seg.samples.append((now, list(q)))
            if len(seg.samples) > MAX_SAMPLES:
                seg.samples = seg.samples[::2]
        still = now - seg.last_change
        if not seg.moved:
            # goal ที่ไม่พาแขนไปไหน (สั่งท่าเดิม) — ทิ้งเมื่อตัวคุมจบหรือรอนานแล้ว
            if (seg.result and still > STILL_S) or now - seg.t_open > 30.0:
                st["seg"] = None
            return
        done = seg.goal is None or seg.result
        if (still >= STILL_S and done) or still >= STILL_MAX_S:
            self.close(arm, now, "นิ่ง")

    def poses(self, arm):
        """ท่าที่สอนไว้ (อ่านไฟล์ใหม่เมื่อไฟล์เปลี่ยน)"""
        import os
        try:
            mtime = os.path.getmtime(programs.DEFAULT_FILE)
            if self.poses_cache[0] != mtime:
                self.poses_cache = (mtime, programs.load(programs.DEFAULT_FILE)["poses"])
        except Exception:
            pass
        return self.poses_cache[1].get(arm, {})

    def close(self, arm, now, why):
        st = self.arms[arm]
        seg = st["seg"]
        st["seg"] = None
        q1 = st["last"]
        st["anchor"] = q1
        samples = seg.samples + [(now, list(q1))]
        deg = math.degrees
        max_speed = [0.0] * 6
        for (ta, qa), (tb, qb) in zip(samples, samples[1:]):
            if tb - ta >= SAMPLE_S * 0.5:
                for k in range(6):
                    max_speed[k] = max(max_speed[k], abs(deg(qb[k] - qa[k])) / (tb - ta))
        poses = self.poses(arm)
        rec = {
            "type": "move", "ts": round(seg.t0, 3), "time": arm_log.local_time(seg.t0), "arm": arm,
            "src": seg.src, "result": seg.result, "dur": round(seg.last_change - seg.t0, 2), "end": why,
            "from": [round(deg(v), 2) for v in seg.q0], "to": [round(deg(v), 2) for v in q1],
            "from_pose": arm_log.pose_name(poses, seg.q0), "to_pose": arm_log.pose_name(poses, q1),
            "max_speed": [round(v, 1) for v in max_speed],
            "j1": self.j1_profile(arm, samples),
        }
        arm_log.append(arm_log.moves_file(seg.t0), rec)
        self.get_logger().info(f"{arm}: {arm_log.move_text(rec)} · {arm_log.j1_text(rec)} · {seg.src} {seg.result}")

    def j1_profile(self, arm, samples):
        if not self.loader or not self.loader.has_arm(arm):
            return None
        try:
            ref = self.loader.reference(arm)
            loads = [self.loader.torque(arm, self.loader.q_of(arm, q)) for _, q in samples]
        except Exception as exc:
            self.get_logger().warn(f"คำนวณภาระ J1 ไม่ได้: {exc}")
            return None
        k = max(range(len(loads)), key=lambda i: abs(loads[i]))
        a, b = max(0, k - 1), min(len(samples) - 1, k + 1)
        dt = samples[b][0] - samples[a][0]
        w = (samples[b][1][0] - samples[a][1][0]) / dt if dt > 0 else 0.0
        lift = None if abs(math.degrees(w)) < 0.5 else bool(loads[k] * w < 0)  # น้ำหนักดึงสวนทางที่ J1 หมุน = ยกขึ้น
        pct = [100.0 * abs(v) / ref for v in loads]
        return {"from": round(pct[0], 1), "to": round(pct[-1], 1), "peak": round(pct[k], 1),
                "peak_at": [round(math.degrees(v), 1) for v in samples[k][1]],
                "speed_at_peak": round(abs(math.degrees(w)), 1), "lift_at_peak": lift}


def programs_speed_note():
    """เพดานความเร็ว J1 ที่ตั้งไว้ตอนนั้น (ไว้ดูย้อนหลังว่า log ช่วงไหนใช้เพดานไหน)"""
    try:
        import os

        import yaml
        from ament_index_python.packages import get_package_share_directory
        with open(os.path.join(get_package_share_directory("ar_dual"), "config", "joint_speed.yaml"),
                  encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return {"board_deg_s": cfg.get("speed"), "board_deg_s2": cfg.get("accel")}
    except Exception:
        return None


def main():
    rclpy.init()
    node = MotionLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
