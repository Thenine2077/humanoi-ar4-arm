#!/usr/bin/env python3
"""มิเรอร์ /joint_states ของแขนจำลอง (sim, namespace เริ่มต้น) ไปสั่งแขนจริง
(namespace 'real') เป็น JointTrajectory จุดเดียวต่อเนื่อง ให้ขยับตามกันแบบเรียลไทม์

  ros2 run ar_mirror joint_mirror
  ros2 run ar_mirror joint_mirror --ros-args -p rate:=10.0 -p time_from_start:=0.3

พารามิเตอร์
  source_topic     (str)   /joint_states                               ที่มา (จากซิม)
  target_topic     (str)   /real/joint_trajectory_controller/joint_trajectory  ปลายทาง (แขนจริง)
  rate             (float) 10.0   Hz สูงสุดที่จะส่งคำสั่งไปแขนจริง
  time_from_start  (float) 0.3    วิ ระยะเวลาที่ให้แขนจริงไปถึงจุดที่สั่ง

⚠️ MoveIt วางแผนเทียบ collision กับฉากในซิมเท่านั้น ไม่รู้จักสิ่งกีดขวางรอบแขนจริง
   เคลียร์พื้นที่รอบแขนจริงให้ปลอดภัยก่อนสั่งเสมอ
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FOLLOWED_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class JointMirror(Node):
    def __init__(self):
        super().__init__("joint_mirror")
        self.declare_parameter("source_topic", "/joint_states")
        self.declare_parameter(
            "target_topic", "/real/joint_trajectory_controller/joint_trajectory"
        )
        # 10 Hz + 0.3s: ยิ่งถี่ แขนจริงยิ่งเกาะตามซิมใกล้ขึ้น (JTC จะ re-target บ่อยขึ้น)
        # เคยต้องบีบเหลือ 5 Hz ตอนวิ่งผ่าน usbip เพราะ serial round-trip กิน 25-43 ms
        # ตอนนี้ใช้สะพาน serial-over-TCP แล้ววัดได้ ~1 ms จึงกลับขึ้นมาได้สบายๆ
        # (หมายเหตุ: ค่านี้ไม่ได้กำหนดภาระ serial — controller_manager เขียน serial ทุกรอบ
        #  ตาม update_rate อยู่แล้วไม่ว่าจะมีคำสั่งใหม่หรือไม่ ดู ar_mirror/config/real_controllers.yaml)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("time_from_start", 0.3)
        # safety interlock: ไม่เริ่มมิเรอร์จนกว่าท่าของซิมกับแขนจริงจะต่างกันไม่เกินค่านี้
        self.declare_parameter("real_state_topic", "/real/joint_states")
        self.declare_parameter("engage_tolerance", 0.26)  # rad (~15 องศา)

        src = str(self.get_parameter("source_topic").value)
        dst = str(self.get_parameter("target_topic").value)
        real_src = str(self.get_parameter("real_state_topic").value)
        self.min_period = 1.0 / float(self.get_parameter("rate").value)
        self.tfs = float(self.get_parameter("time_from_start").value)
        self.tol = float(self.get_parameter("engage_tolerance").value)

        self.pub = self.create_publisher(JointTrajectory, dst, 10)
        self.sub = self.create_subscription(JointState, src, self._on_state, 10)
        self.real_sub = self.create_subscription(
            JointState, real_src, self._on_real_state, 10
        )
        self.last_t = 0.0
        self.real_positions = None
        self.engaged = False
        self.get_logger().info(
            f"joint_mirror พร้อม — {src} -> {dst} สูงสุด {1.0 / self.min_period:.0f} Hz "
            f"(time_from_start={self.tfs:.2f}s) | รอให้ท่าตรงกันภายใน "
            f"{self.tol:.2f} rad ก่อนเริ่มสั่งแขนจริง"
        )

    def _on_real_state(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        try:
            self.real_positions = [by_name[j] for j in FOLLOWED_JOINTS]
        except KeyError:
            pass

    def _on_state(self, msg: JointState):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_t < self.min_period:
            return

        by_name = dict(zip(msg.name, msg.position))
        try:
            positions = [by_name[j] for j in FOLLOWED_JOINTS]
        except KeyError:
            return  # ยังไม่มีครบ 6 แกน (เช่นตอนเพิ่ง spawn)

        if not self.engaged:
            if self.real_positions is None:
                self.get_logger().warn(
                    "ยังไม่ได้รับ joint_states ของแขนจริง — ยังไม่เริ่มมิเรอร์",
                    throttle_duration_sec=3.0,
                )
                return
            diffs = [abs(s - r) for s, r in zip(positions, self.real_positions)]
            worst = max(diffs)
            if worst > self.tol:
                worst_joint = FOLLOWED_JOINTS[diffs.index(worst)]
                self.get_logger().warn(
                    f"ท่าซิมกับแขนจริงต่างกันมากเกินไป ({worst_joint} ต่าง "
                    f"{worst:.3f} rad = {worst * 57.3:.1f} องศา, เกณฑ์ {self.tol:.2f} rad) "
                    "— ยังไม่สั่งแขนจริง กันกระชาก ให้ขยับซิมไปใกล้ท่าแขนจริงก่อน",
                    throttle_duration_sec=3.0,
                )
                return
            self.engaged = True
            self.get_logger().info(
                f"ท่าตรงกันแล้ว (ต่างสูงสุด {worst:.3f} rad) — เริ่มมิเรอร์ไปแขนจริง"
            )

        self.last_t = now
        traj = JointTrajectory()
        traj.joint_names = FOLLOWED_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(seconds=self.tfs).to_msg()
        traj.points = [pt]
        self.pub.publish(traj)


def main():
    rclpy.init()
    node = JointMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
