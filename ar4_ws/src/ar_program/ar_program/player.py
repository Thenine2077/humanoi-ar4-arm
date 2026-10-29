#!/usr/bin/env python3
"""arm_program — เล่นโปรแกรมท่าที่ตั้งไว้ล่วงหน้า (แขนเดียวหรือสองแขนพร้อมกัน) เมื่อมีคนส่งชื่อโปรแกรมมาทาง ROS

  ros2 run ar_program arm_program
  ros2 topic pub --once /arm/run  std_msgs/msg/String "{data: demo}"   # สั่งรัน
  ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}"              # หยุดกลางคัน
  ros2 topic echo /arm/done                                            # ผล: demo:ok / demo:error:...

หัวข้อ
  /arm/run   (sub, String)  ชื่อโปรแกรมใน programs.yaml, "@ท่า" (แขนซ้าย) หรือ "@right:ท่า" = ไปท่าเดียว
  /arm/stop  (sub, Empty)   ยกเลิกโปรแกรมที่กำลังรัน (ทุกแขนหยุดที่ตำแหน่งปัจจุบัน)
  /arm/done  (pub, String)  "<โปรแกรม>:ok" | ":error:<เหตุผล>" | ":stopped" | ":rejected:busy"
  /arm/state (pub, String)  "idle" | "running:<โปรแกรม>:<ขั้นที่>/<ทั้งหมด>:<ซ้าย>/<ขวา>[ มือ <ท่า>]"
  /hand/gesture       (pub, String)  ท่ามือของมือบนแขนซ้าย (amazinghand_node ใน humanoi_games)
  /right_hand/gesture (pub, String)  ท่ามือของมือบนแขนขวา (ยังไม่มีมือ — ถ้าไม่มีใครฟังจะเตือนแล้วข้าม)

พารามิเตอร์
  programs_file       ไฟล์ท่า/โปรแกรม (อ่านใหม่ทุกครั้งที่สั่งรัน แก้ไฟล์แล้วไม่ต้องรีสตาร์ท)
  layout              dual (ค่าเริ่ม: ar_dual สองแขน ข้อต่อ left_joint_N/right_joint_N ตัวคุม <แขน>_arm_controller)
                      | single (แขนเดี่ยวแบบเดิม ข้อต่อ joint_N ใช้ action_name) — single มีแต่แขนซ้าย
  action_name         เฉพาะ single: /joint_trajectory_controller/follow_joint_trajectory
                      (แขนจริงในชุด dual_real_sim ใช้ /real/joint_trajectory_controller/...)
  joint_states_topic  /joint_states  (แขนจริงในชุด dual_real_sim ใช้ /real/joint_states)
  sync                true = แขนที่ขยับในขั้นเดียวกันถึงพร้อมกัน (แขนที่ไปใกล้กว่าวิ่งช้าลง)
  speed               ความเร็วช่วงวิ่งคงที่ของข้อต่อที่ไปไกลสุด rad/s (ค่าเริ่ม 0.4 แต่ถูกจำกัดที่ max_speed)
  accel               อัตราเร่ง/เบรก rad/s^2 (0.08 ≈ 4.6°/s^2 — สายพาน J1 เคยรูดตอนแขนยื่นไกลแล้วเร่ง/หยุดแรง)
  max_speed           เพดานของ speed (0.19 = ความเร็ว J1 ตอน calibrate) — บอร์ดจำกัด J1 ไว้ 11.25°/s
                      (ar_dual/config/joint_speed.yaml) สั่งเร็วกว่านี้แขนตามไม่ทันจนเกิน path tolerance
                      0.2 rad ของ controller แล้วโดน abort กลางทาง (เดิม 0.22 ก่อนจำกัด J1 ในบอร์ด)
  min_move_time       เวลาขั้นต่ำต่อท่า วินาที (กันสั่งกระชากตอนท่าห่างกันนิดเดียว)

ทุกขั้น แขนที่มีท่าเริ่มพร้อมกัน แขนที่ still ไม่ถูกสั่ง (ค้างอยู่ที่เดิม) ขั้นถัดไปเริ่มเมื่อทุกแขนถึงแล้ว
ท่ามือในขั้นส่งตอนเริ่มขั้น (ขยับพร้อมแขน) ขั้นจบเมื่อแขนถึงและมือใช้เวลาครบ (programs.HAND_GESTURES)
ทุกท่าส่งตรงเข้า joint_trajectory_controller เป็นเส้นตรงในปริภูมิข้อต่อ (เหมือน Move J ของ HMI)
ด้วยโปรไฟล์ เร่ง → ความเร็วคงที่ → เบรก (programs.trapezoid) ไม่ผ่าน MoveIt จึงไม่มีการเช็คการชน
— รวมถึงสองแขนชนกันเอง/ชนเสา ผลออกมาเหมือนเดิมทุกรอบ ทดสอบโปรแกรมช้าๆ ก่อนเสมอ
"""
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint

from ar_program import programs


def _wait(future, timeout):
    """รอ future ของ rclpy จากเธรดอื่น (executor หมุนอยู่ในเธรดหลัก)"""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        return None
    return future.result()


def _duration(secs):
    return Duration(sec=int(secs), nanosec=int((secs - int(secs)) * 1e9))


ARM_TH = {"left": "ซ้าย", "right": "ขวา"}


class ArmProgram(Node):
    def __init__(self):
        super().__init__("arm_program")
        self.declare_parameter("programs_file", programs.DEFAULT_FILE)
        self.declare_parameter("layout", "dual")
        self.declare_parameter("action_name", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("sync", True)
        self.declare_parameter("speed", 0.4)
        self.declare_parameter("accel", 0.08)
        self.declare_parameter("max_speed", 0.19)
        self.declare_parameter("min_move_time", 1.0)
        self.declare_parameter("left_hand_topic", "/hand/gesture")
        self.declare_parameter("right_hand_topic", "/right_hand/gesture")

        cb = ReentrantCallbackGroup()
        self.layout = self.get_parameter("layout").value
        if self.layout == "dual":
            actions = {arm: f"/{arm}_arm_controller/follow_joint_trajectory" for arm in programs.ARMS}
        else:
            actions = {"left": self.get_parameter("action_name").value}
        self.arms = tuple(actions)
        self.action_clients = {arm: ActionClient(self, FollowJointTrajectory, name, callback_group=cb)
                        for arm, name in actions.items()}
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10,
            callback_group=cb)
        self.create_subscription(String, "/arm/run", self._on_run, 10, callback_group=cb)
        self.create_subscription(Empty, "/arm/stop", self._on_stop, 10, callback_group=cb)
        self.hand_pubs = {arm: self.create_publisher(String, self.get_parameter(f"{arm}_hand_topic").value, 10)
                          for arm in programs.ARMS}
        self.done_pub = self.create_publisher(String, "/arm/done", 10)
        self.state_pub = self.create_publisher(String, "/arm/state", 10)

        self.current = {}
        self.lock = threading.Lock()
        self.busy = False
        self.stop_flag = threading.Event()
        self.goal_handles = []
        self._state("idle")
        self.get_logger().info(
            f"arm_program พร้อม ({self.layout}) — {', '.join(actions.values())} | "
            f"ไฟล์ {self.get_parameter('programs_file').value}")

    # ---------- callbacks ----------
    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def _on_run(self, msg):
        name = msg.data.strip()
        with self.lock:
            if self.busy:
                self.get_logger().warn(f"ปฏิเสธ '{name}' — กำลังรันโปรแกรมอื่นอยู่")
                self._done(f"{name}:rejected:busy")
                return
            self.busy = True
            self.stop_flag.clear()
        threading.Thread(target=self._run, args=(name,), daemon=True).start()

    def _on_stop(self, _msg):
        if not self.busy:
            return
        self.get_logger().warn("ได้รับ /arm/stop — ยกเลิกโปรแกรม")
        self.stop_flag.set()
        for gh in list(self.goal_handles):
            gh.cancel_goal_async()

    # ---------- execution (worker thread) ----------
    def _run(self, name):
        try:
            try:
                result = self._execute(name)
            except Exception as exc:  # ไม่ให้เธรดตายเงียบ — ต้องรายงานกลับเสมอ
                result = f"error:{exc}"
            self._done(f"{name}:{result}")
            # แยก info/error คนละบรรทัด: rclpy จำระดับ log ต่อ "บรรทัดที่เรียก" ถ้าบรรทัดเดียวกัน
            # พิมพ์คนละระดับจะ raise ValueError (เคยทำให้เธรดตายแล้ว busy ค้าง True ถาวร)
            if result == "ok":
                self.get_logger().info(f"โปรแกรม '{name}' จบ: ok")
            else:
                self.get_logger().error(f"โปรแกรม '{name}' จบ: {result}")
        finally:
            # ปลด busy ใน finally เสมอ — ถ้าค้าง True ทุกคำสั่งหลังจากนี้จะโดน rejected:busy หมด
            self.goal_handles = []
            self._state("idle")
            with self.lock:
                self.busy = False

    def _execute(self, name):
        path = self.get_parameter("programs_file").value
        try:
            steps = programs.resolve(programs.load(path), name, self.arms)
        except (programs.ProgramError, OSError) as exc:
            return f"error:{exc}"

        used = {arm for kind, value, _ in steps if kind == "move" for arm in value}
        for arm in sorted({arm for _, _, hands in steps for arm in hands}):
            if not self.hand_pubs[arm].get_subscription_count():
                self.get_logger().warn(f"ไม่มีตัวคุมมือบนแขน{ARM_TH[arm]} ฟังอยู่ "
                                       f"({self.hand_pubs[arm].topic_name}) — ท่ามือของแขนนี้จะถูกข้าม")
        for arm in used:
            if not self.action_clients[arm].wait_for_server(timeout_sec=3.0):
                return f"error:ไม่พบตัวคุมแขน{ARM_TH[arm]} (ระบบแขนเปิดอยู่ไหม)"
            if not all(j in self.current for j in programs.joint_names(arm, self.layout)):
                return f"error:ยังไม่ได้รับ joint_states ของแขน{ARM_TH[arm]} ครบ 6 ข้อต่อ"

        speed = float(self.get_parameter("speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        if speed > max_speed:
            self.get_logger().warn(
                f"speed {speed:.2f} เกินเพดาน {max_speed:.2f} rad/s — ใช้ {max_speed:.2f} "
                "(เร็วกว่านี้เฟิร์มแวร์ตามไม่ทัน controller จะ abort)")
            speed = max_speed
        accel = float(self.get_parameter("accel").value)
        min_time = float(self.get_parameter("min_move_time").value)
        sync = bool(self.get_parameter("sync").value)
        self.get_logger().info(f"เริ่มโปรแกรม '{name}' ({len(steps)} ขั้น)")

        for i, (kind, value, hands) in enumerate(steps, 1):
            if self.stop_flag.is_set():
                return "stopped"
            if kind == "wait":
                self._state(f"running:{name}:{i}/{len(steps)}:wait {value:.1f}s")
                end = time.monotonic() + value
                while time.monotonic() < end:
                    if self.stop_flag.is_set():
                        return "stopped"
                    time.sleep(0.05)
                continue

            self._state(f"running:{name}:{i}/{len(steps)}:" + "/".join(
                value[arm][0] if arm in value else "-" for arm in self.arms)
                + "".join(f" มือ{ARM_TH[arm]} {g}" for arm, g in hands.items()))
            step_start = time.monotonic()
            for arm, gesture in hands.items():  # มือขยับพร้อมแขน
                self.hand_pubs[arm].publish(String(data=gesture))
            hand_end = step_start + max([programs.hand_duration(g) for g in hands.values()] or [0.0])
            if not value:  # ขั้นมืออย่างเดียว
                self.get_logger().info(f"  ขั้น {i}/{len(steps)}: " + " · ".join(
                    f"มือ{ARM_TH[arm]} -> {g}" for arm, g in hands.items()))
                if not self._sleep_until(hand_end):
                    return "stopped"
                continue
            starts = {arm: [self.current[j] for j in programs.joint_names(arm, self.layout)] for arm in value}
            plans = {arm: programs.trapezoid(starts[arm], target, speed, accel, min_time)
                     for arm, (_, target) in value.items()}
            if sync and len(plans) > 1:  # ให้ถึงพร้อมกัน: ยืดแขนที่เร็วกว่าให้ใช้เวลาเท่าแขนที่ช้าที่สุด
                longest = max(p[-1][0] for p in plans.values())
                plans = {arm: programs.trapezoid(starts[arm], target, speed, accel, longest)
                         for arm, (_, target) in value.items()}
            self.get_logger().info(f"  ขั้น {i}/{len(steps)}: " + " · ".join(
                f"{ARM_TH[arm]} -> '{pose}' "
                f"{math.degrees(max(abs(t - s) for t, s in zip(target, starts[arm]))):.1f}° "
                f"{plans[arm][-1][0]:.1f} s" for arm, (pose, target) in value.items())
                + "".join(f" · มือ{ARM_TH[arm]} -> {g}" for arm, g in hands.items()))
            err = self._move(plans)
            if err:
                return err
            if not self._sleep_until(hand_end):  # แขนถึงก่อนมือทำท่าเสร็จ (เช่นท่าเคลื่อนไหว) — รอมือ
                return "stopped"
        return "ok"

    def _sleep_until(self, t):
        """รอถึงเวลา t — คืน False ถ้าโดนสั่งหยุดระหว่างรอ"""
        while time.monotonic() < t:
            if self.stop_flag.is_set():
                return False
            time.sleep(0.02)
        return True

    def _move(self, plans):
        """ส่งทุกแขนพร้อมกัน แล้วรอจนครบทุกแขน — แขนไหนพังให้หยุดแขนที่เหลือด้วย"""
        futures = {}
        for arm, points in plans.items():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = programs.joint_names(arm, self.layout)
            for t, positions, velocities in points:
                pt = JointTrajectoryPoint()
                pt.positions = list(positions)
                pt.velocities = list(velocities)
                pt.time_from_start = _duration(t)
                goal.trajectory.points.append(pt)
            futures[arm] = self.action_clients[arm].send_goal_async(goal)
        accepted = {}
        for arm, fut in futures.items():
            gh = _wait(fut, 5.0)
            if gh is None or not gh.accepted:
                for other in accepted.values():
                    other.cancel_goal_async()
                return f"error:ตัวคุมแขน{ARM_TH[arm]} ไม่รับคำสั่ง (goal rejected)"
            accepted[arm] = gh
        self.goal_handles = list(accepted.values())
        secs = max(points[-1][0] for points in plans.values())
        for arm, gh in accepted.items():
            res = _wait(gh.get_result_async(), secs + 10.0)
            if self.stop_flag.is_set():
                return "stopped"
            if res is None or res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                for other in accepted.values():
                    other.cancel_goal_async()
                if res is None:
                    return f"error:แขน{ARM_TH[arm]} รอผลการเคลื่อนที่เกินเวลา"
                return (f"error:ตัวคุมแขน{ARM_TH[arm]} รายงาน error {res.result.error_code} "
                        f"{res.result.error_string}").strip()
        return None

    # ---------- helpers ----------
    def _done(self, text):
        self.done_pub.publish(String(data=text))

    def _state(self, text):
        self.state_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = ArmProgram()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
