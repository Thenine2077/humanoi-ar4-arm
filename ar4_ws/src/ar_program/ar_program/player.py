#!/usr/bin/env python3
"""arm_program — plays preset pose programs (one arm or both arms together) when someone sends a program name over ROS

  ros2 run ar_program arm_program
  ros2 topic pub --once /arm/run  std_msgs/msg/String "{data: demo}"   # run
  ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}"              # stop midway
  ros2 topic echo /arm/done                                            # result: demo:ok / demo:error:...

Topics
  /arm/run   (sub, String)  program name in programs.yaml, "@pose" (left arm) or "@right:pose" = go to a single pose
  /arm/stop  (sub, Empty)   cancel the running program (every arm stops where it is)
  /arm/done  (pub, String)  "<program>:ok" | ":error:<reason>" | ":stopped" | ":rejected:busy"
  /arm/state (pub, String)  "idle" | "running:<program>:<step>/<total>:<left>/<right>[ <arm> hand <gesture>]"
  /hand/gesture       (pub, String)  hand gesture for the hand on the left arm (amazinghand_node in humanoi_games)
  /right_hand/gesture (pub, String)  hand gesture for the hand on the right arm (no hand yet — if nobody listens, warn and skip)

Parameters
  programs_file       pose/program file (re-read on every run; no restart needed after editing)
  layout              dual (default: ar_dual two arms, joints left_joint_N/right_joint_N, controllers <arm>_arm_controller)
                      | single (old single-arm setup, joints joint_N, uses action_name) — single has the left arm only
  action_name         single only: /joint_trajectory_controller/follow_joint_trajectory
                      (the real arm in the dual_real_sim set uses /real/joint_trajectory_controller/...)
  joint_states_topic  /joint_states  (the real arm in the dual_real_sim set uses /real/joint_states)
  sync                true = arms moving in the same step arrive together (the arm with less to travel runs slower)
  speed               cruise speed of the joint travelling furthest, rad/s (default 0.4 but capped at max_speed)
  accel               accel/brake rate rad/s^2 (0.08 ≈ 4.6°/s^2 — the J1 belt has slipped with the arm reaching far out under hard accel/stops)
  max_speed           cap on speed (0.19 = J1 speed during calibration) — the board limits J1 to 11.25°/s
                      (ar_dual/config/joint_speed.yaml); faster commands leave the arm lagging beyond the path tolerance
                      0.2 rad of the controller and it gets aborted midway (was 0.22 before J1 was limited in the board)
  min_move_time       minimum time per pose in seconds (avoids jerky moves when poses are very close)

In every step the arms with a pose start together; arms set to still get no command (stay put); the next step starts once every arm has arrived
Hand gestures in a step are sent when the step starts (moving with the arm); a step ends when the arms have arrived and the hands have had their time (programs.HAND_GESTURES)
Every pose goes straight to joint_trajectory_controller as a straight line in joint space (like Move J on the HMI)
with an accelerate → cruise → brake profile (programs.trapezoid); it does not go through MoveIt, so there is no collision check
— including the two arms hitting each other / the post; the result is the same every run, so always test programs slowly first
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
    """Wait for an rclpy future from another thread (the executor spins in the main thread)"""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        return None
    return future.result()


def _duration(secs):
    return Duration(sec=int(secs), nanosec=int((secs - int(secs)) * 1e9))


ARM_TH = {"left": "left", "right": "right"}


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
            f"arm_program ready ({self.layout}) — {', '.join(actions.values())} | "
            f"file {self.get_parameter('programs_file').value}")

    # ---------- callbacks ----------
    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def _on_run(self, msg):
        name = msg.data.strip()
        with self.lock:
            if self.busy:
                self.get_logger().warn(f"Rejected '{name}' — another program is running")
                self._done(f"{name}:rejected:busy")
                return
            self.busy = True
            self.stop_flag.clear()
        threading.Thread(target=self._run, args=(name,), daemon=True).start()

    def _on_stop(self, _msg):
        if not self.busy:
            return
        self.get_logger().warn("Got /arm/stop — cancelling the program")
        self.stop_flag.set()
        for gh in list(self.goal_handles):
            gh.cancel_goal_async()

    # ---------- execution (worker thread) ----------
    def _run(self, name):
        try:
            try:
                result = self._execute(name)
            except Exception as exc:  # don't let the thread die silently — always report back
                result = f"error:{exc}"
            self._done(f"{name}:{result}")
            # info/error on separate lines: rclpy remembers the log level per "calling line"; if the same line
            # logs at different levels it raises ValueError (this once killed the thread and left busy stuck at True forever)
            if result == "ok":
                self.get_logger().info(f"Program '{name}' finished: ok")
            else:
                self.get_logger().error(f"Program '{name}' finished: {result}")
        finally:
            # always clear busy in finally — if it stays True, every later command gets rejected:busy
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
                self.get_logger().warn(f"No hand controller listening on the {ARM_TH[arm]} arm "
                                       f"({self.hand_pubs[arm].topic_name}) — this arm's hand gestures will be skipped")
        for arm in used:
            if not self.action_clients[arm].wait_for_server(timeout_sec=3.0):
                return f"error:{ARM_TH[arm]} arm controller not found (is the arm system on?)"
            if not all(j in self.current for j in programs.joint_names(arm, self.layout)):
                return f"error:joint_states for all 6 joints of the {ARM_TH[arm]} arm not received yet"

        speed = float(self.get_parameter("speed").value)
        max_speed = float(self.get_parameter("max_speed").value)
        if speed > max_speed:
            self.get_logger().warn(
                f"speed {speed:.2f} is over the cap {max_speed:.2f} rad/s — using {max_speed:.2f} "
                "(faster than this the firmware can't keep up and the controller aborts)")
            speed = max_speed
        accel = float(self.get_parameter("accel").value)
        min_time = float(self.get_parameter("min_move_time").value)
        sync = bool(self.get_parameter("sync").value)
        self.get_logger().info(f"Starting program '{name}' ({len(steps)} steps)")

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
                + "".join(f" {ARM_TH[arm]} hand {g}" for arm, g in hands.items()))
            step_start = time.monotonic()
            for arm, gesture in hands.items():  # hands move together with the arms
                self.hand_pubs[arm].publish(String(data=gesture))
            hand_end = step_start + max([programs.hand_duration(g) for g in hands.values()] or [0.0])
            if not value:  # hand-only step
                self.get_logger().info(f"  step {i}/{len(steps)}: " + " · ".join(
                    f"{ARM_TH[arm]} hand -> {g}" for arm, g in hands.items()))
                if not self._sleep_until(hand_end):
                    return "stopped"
                continue
            starts = {arm: [self.current[j] for j in programs.joint_names(arm, self.layout)] for arm in value}
            plans = {arm: programs.trapezoid(starts[arm], target, speed, accel, min_time)
                     for arm, (_, target) in value.items()}
            if sync and len(plans) > 1:  # arrive together: stretch the faster arms to take as long as the slowest
                longest = max(p[-1][0] for p in plans.values())
                plans = {arm: programs.trapezoid(starts[arm], target, speed, accel, longest)
                         for arm, (_, target) in value.items()}
            self.get_logger().info(f"  step {i}/{len(steps)}: " + " · ".join(
                f"{ARM_TH[arm]} -> '{pose}' "
                f"{math.degrees(max(abs(t - s) for t, s in zip(target, starts[arm]))):.1f}° "
                f"{plans[arm][-1][0]:.1f} s" for arm, (pose, target) in value.items())
                + "".join(f" · {ARM_TH[arm]} hand -> {g}" for arm, g in hands.items()))
            err = self._move(plans)
            if err:
                return err
            if not self._sleep_until(hand_end):  # arms arrived before the hand finished its gesture (e.g. a moving gesture) — wait for the hand
                return "stopped"
        return "ok"

    def _sleep_until(self, t):
        """Wait until time t — returns False if stopped while waiting"""
        while time.monotonic() < t:
            if self.stop_flag.is_set():
                return False
            time.sleep(0.02)
        return True

    def _move(self, plans):
        """Send to every arm at once, then wait for all of them — if one arm fails, stop the others too"""
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
                return f"error:{ARM_TH[arm]} arm controller did not accept the goal (goal rejected)"
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
                    return f"error:{ARM_TH[arm]} arm: timed out waiting for the motion result"
                return (f"error:{ARM_TH[arm]} arm controller reported error {res.result.error_code} "
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
