#!/usr/bin/env python3
"""Mirror /joint_states of the simulated arm (sim, default namespace) to command the real arm
(namespace 'real') as a continuous single-point JointTrajectory, so they move together in real time

  ros2 run ar_mirror joint_mirror
  ros2 run ar_mirror joint_mirror --ros-args -p rate:=10.0 -p time_from_start:=0.3

Parameters
  source_topic     (str)   /joint_states                               source (from the sim)
  target_topic     (str)   /real/joint_trajectory_controller/joint_trajectory  destination (real arm)
  rate             (float) 10.0   max Hz for sending commands to the real arm
  time_from_start  (float) 0.3    s, time given to the real arm to reach the commanded point

⚠️ MoveIt checks collisions against the sim scene only; it knows nothing about obstacles around the real arm
   Always clear the area around the real arm before sending commands
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
        # 10 Hz + 0.3s: the more frequent, the closer the real arm follows the sim (the JTC re-targets more often)
        # It once had to be cut to 5 Hz when running over usbip, because a serial round-trip took 25-43 ms
        # Now the serial-over-TCP bridge measures ~1 ms, so it can easily go back up
        # (Note: this value does not set the serial load — controller_manager writes serial every cycle
        #  at update_rate anyway, new command or not; see ar_mirror/config/real_controllers.yaml)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("time_from_start", 0.3)
        # safety interlock: don't start mirroring until the sim pose and the real arm differ by no more than this
        self.declare_parameter("real_state_topic", "/real/joint_states")
        self.declare_parameter("engage_tolerance", 0.26)  # rad (~15 degrees)

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
            f"joint_mirror ready — {src} -> {dst} max {1.0 / self.min_period:.0f} Hz "
            f"(time_from_start={self.tfs:.2f}s) | waiting for the poses to match within "
            f"{self.tol:.2f} rad before commanding the real arm"
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
            return  # not all 6 axes yet (e.g. right after spawn)

        if not self.engaged:
            if self.real_positions is None:
                self.get_logger().warn(
                    "No joint_states from the real arm yet — mirroring not started",
                    throttle_duration_sec=3.0,
                )
                return
            diffs = [abs(s - r) for s, r in zip(positions, self.real_positions)]
            worst = max(diffs)
            if worst > self.tol:
                worst_joint = FOLLOWED_JOINTS[diffs.index(worst)]
                self.get_logger().warn(
                    f"Sim pose and real arm differ too much ({worst_joint} off by "
                    f"{worst:.3f} rad = {worst * 57.3:.1f} degrees, limit {self.tol:.2f} rad) "
                    "— not commanding the real arm, to avoid a jerk; move the sim close to the real arm's pose first",
                    throttle_duration_sec=3.0,
                )
                return
            self.engaged = True
            self.get_logger().info(
                f"Poses match (max difference {worst:.3f} rad) — mirroring to the real arm"
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
