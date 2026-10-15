#!/usr/bin/env python3
"""arm_program — เล่นโปรแกรมท่าที่ตั้งไว้ล่วงหน้า เมื่อมีคนส่งชื่อโปรแกรมมาทาง ROS

  ros2 run ar_program arm_program
  ros2 topic pub --once /arm/run  std_msgs/msg/String "{data: demo}"   # สั่งรัน
  ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}"              # หยุดกลางคัน
  ros2 topic echo /arm/done                                            # ผล: demo:ok / demo:error:...

หัวข้อ
  /arm/run   (sub, String)  ชื่อโปรแกรมใน programs.yaml
  /arm/stop  (sub, Empty)   ยกเลิกโปรแกรมที่กำลังรัน (แขนหยุดที่ตำแหน่งปัจจุบัน)
  /arm/done  (pub, String)  "<โปรแกรม>:ok" | ":error:<เหตุผล>" | ":stopped" | ":rejected:busy"
  /arm/state (pub, String)  "idle" | "running:<โปรแกรม>:<ขั้นที่>/<ทั้งหมด>:<ท่า>"

พารามิเตอร์
  programs_file       ไฟล์ท่า/โปรแกรม (อ่านใหม่ทุกครั้งที่สั่งรัน แก้ไฟล์แล้วไม่ต้องรีสตาร์ท)
  action_name         /joint_trajectory_controller/follow_joint_trajectory
                      (แขนจริงในชุด dual_real_sim ใช้ /real/joint_trajectory_controller/...)
  joint_states_topic  /joint_states  (แขนจริงในชุด dual ใช้ /real/joint_states)
  speed               ความเร็วช่วงวิ่งคงที่ของข้อต่อที่ไปไกลสุด rad/s (ค่าเริ่ม 0.4 แต่ถูกจำกัดที่ max_speed)
  accel               อัตราเร่ง/เบรก rad/s^2 (0.08 ≈ 4.6°/s^2 — สายพาน J1 เคยรูดตอนแขนยื่นไกลแล้วเร่ง/หยุดแรง)
  max_speed           เพดานของ speed (0.22) — เฟิร์มแวร์ตามหลังเป้าราว v^2/(2·10°/s^2) ถ้าเกิน 0.22
                      จะตามหลังเกิน path tolerance 0.2 rad ของ controller แล้วโดน abort กลางทาง
  min_move_time       เวลาขั้นต่ำต่อท่า วินาที (กันสั่งกระชากตอนท่าห่างกันนิดเดียว)

ทุกท่าส่งตรงเข้า joint_trajectory_controller เป็นเส้นตรงในปริภูมิข้อต่อ (เหมือน Move J ของ HMI)
ด้วยโปรไฟล์ เร่ง → ความเร็วคงที่ → เบรก (programs.trapezoid) ไม่ผ่าน MoveIt จึงไม่มีการเช็คการชน
ผลออกมาเหมือนเดิมทุกรอบ ทดสอบโปรแกรมช้าๆ ก่อนเสมอ
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


class ArmProgram(Node):
    def __init__(self):
        super().__init__("arm_program")
        self.declare_parameter("programs_file", programs.DEFAULT_FILE)
        self.declare_parameter("action_name", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("speed", 0.4)
        self.declare_parameter("accel", 0.08)
        self.declare_parameter("max_speed", 0.22)
        self.declare_parameter("min_move_time", 1.0)

        cb = ReentrantCallbackGroup()
        action = self.get_parameter("action_name").value
        self.client = ActionClient(self, FollowJointTrajectory, action, callback_group=cb)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value, self._on_js, 10,
            callback_group=cb)
        self.create_subscription(String, "/arm/run", self._on_run, 10, callback_group=cb)
        self.create_subscription(Empty, "/arm/stop", self._on_stop, 10, callback_group=cb)
        self.done_pub = self.create_publisher(String, "/arm/done", 10)
        self.state_pub = self.create_publisher(String, "/arm/state", 10)

        self.current = {}
        self.lock = threading.Lock()
        self.busy = False
        self.stop_flag = threading.Event()
        self.goal_handle = None
        self._state("idle")
        self.get_logger().info(
            f"arm_program พร้อม — action {action} | ไฟล์ {self.get_parameter('programs_file').value}")

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
        gh = self.goal_handle
        if gh is not None:
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
            self.goal_handle = None
            self._state("idle")
            with self.lock:
                self.busy = False

    def _execute(self, name):
        path = self.get_parameter("programs_file").value
        try:
            steps = programs.resolve(programs.load(path), name)
        except (programs.ProgramError, OSError) as exc:
            return f"error:{exc}"

        if not self.client.wait_for_server(timeout_sec=3.0):
            return "error:ไม่พบ action server ของ joint_trajectory_controller (ซิม/แขนเปิดอยู่ไหม)"
        if not all(j in self.current for j in programs.JOINTS):
            return "error:ยังไม่ได้รับ joint_states ครบ 6 ข้อต่อ"

        speed = float(self.get_parameter("speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        if speed > max_speed:
            self.get_logger().warn(
                f"speed {speed:.2f} เกินเพดาน {max_speed:.2f} rad/s — ใช้ {max_speed:.2f} "
                "(เร็วกว่านี้เฟิร์มแวร์ตามไม่ทัน controller จะ abort)")
            speed = max_speed
        accel = float(self.get_parameter("accel").value)
        min_time = float(self.get_parameter("min_move_time").value)
        self.get_logger().info(f"เริ่มโปรแกรม '{name}' ({len(steps)} ขั้น)")

        for i, (kind, pose, value) in enumerate(steps, 1):
            if self.stop_flag.is_set():
                return "stopped"
            label = pose if kind == "pose" else f"wait {value:.1f}s"
            self._state(f"running:{name}:{i}/{len(steps)}:{label}")

            if kind == "wait":
                end = time.monotonic() + value
                while time.monotonic() < end:
                    if self.stop_flag.is_set():
                        return "stopped"
                    time.sleep(0.05)
                continue

            start = [self.current[j] for j in programs.JOINTS]
            biggest = max(abs(t - s) for t, s in zip(value, start))
            points = programs.trapezoid(start, value, speed, accel, min_time)
            secs = points[-1][0]
            self.get_logger().info(
                f"  ขั้น {i}/{len(steps)}: ท่า '{pose}' ระยะไกลสุด "
                f"{math.degrees(biggest):.1f}° ใช้เวลา {secs:.1f} s")
            err = self._move(points)
            if err:
                return err
        return "ok"

    def _move(self, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(programs.JOINTS)
        for t, positions, velocities in points:
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.velocities = list(velocities)
            pt.time_from_start = _duration(t)
            goal.trajectory.points.append(pt)
        secs = points[-1][0]

        gh = _wait(self.client.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            return "error:controller ไม่รับคำสั่ง (goal rejected)"
        self.goal_handle = gh
        res = _wait(gh.get_result_async(), secs + 10.0)
        if self.stop_flag.is_set():
            return "stopped"
        if res is None:
            return "error:รอผลการเคลื่อนที่เกินเวลา"
        code = res.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            return f"error:controller รายงาน error {code} {res.result.error_string}".strip()
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
