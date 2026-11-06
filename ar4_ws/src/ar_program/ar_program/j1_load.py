"""Arm weight load at joint 1 (J1) — gravity torque about the J1 axis from every part that turns with J1, compared with calibration

The arms hang on the sides of the torso with the J1 axis horizontal, so J1 lifts the whole arm (+ hand) like a weight: the further the arm reaches from the axis, the heavier
The J1 belt has slipped with the arm reaching far out, while during calibration (other joints at the rest pose, arm pointing sideways along the J1 axis)
J1 swings through horizontal and has never slipped -> the peak load there is used as 100%

  load % = |torque at that pose| / peak torque while J1 sweeps its full range with joints 2-6 = 0 × 100
  over 100% = heavier than what has been proven not to slip

Masses come from the URDF (ar_arm.xacro = AR3 values + 0.4 kg hand); real weights differ, so only relative percentages can be trusted,
not N·m — if a part is much heavier than in the model (e.g. the joint 4-6 motors) the numbers skew that way
No ROS needed (only reads the URDF) — urdf_text() gets the URDF from the /robot_description topic or the ar_dual xacro
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

G = np.array([0.0, 0.0, -9.81])  # in the torso frame (z up)
SWEEP_STEPS = 72  # full J1 range ~340° in ~5° steps (peak off by less than 0.1%)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _origin(el):
    o = el.find("origin")
    xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
    rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


class J1Load:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.joints = {j.get("name"): j for j in root.findall("joint")}
        self.links = {lk.get("name"): lk for lk in root.findall("link")}
        self.children = {}
        childs = set()
        for j in self.joints.values():
            self.children.setdefault(j.find("parent").get("link"), []).append(j)
            childs.add(j.find("child").get("link"))
        self.root = next(n for n in self.links if n not in childs)
        self._ref = {}
        self._mass = {}

    def has_arm(self, arm):
        return f"{arm}_joint_1" in self.joints

    def _world(self, q):
        out, stack = {self.root: np.eye(4)}, [self.root]
        while stack:
            p = stack.pop()
            for j in self.children.get(p, []):
                T = out[p] @ _origin(j)
                if j.get("type") in ("revolute", "continuous"):
                    ax = [float(v) for v in j.find("axis").get("xyz").split()]
                    R = np.eye(4)
                    R[:3, :3] = _axis_angle(ax, q.get(j.get("name"), 0.0))
                    T = T @ R
                c = j.find("child").get("link")
                out[c] = T
                stack.append(c)
        return out

    def _masses(self, arm):
        """[(link, kg, centre of mass in the link frame)] of the parts that turn with J1 — link_1..6 + hand (not the arm base)"""
        if arm not in self._mass:
            res = []
            for name, lk in self.links.items():
                if not name.startswith(arm + "_") or name == f"{arm}_base_link":
                    continue
                ine = lk.find("inertial")
                if ine is None:
                    continue
                res.append((name, float(ine.find("mass").get("value")), _origin(ine)[:3, 3]))
            self._mass[arm] = res
        return self._mass[arm]

    def torque(self, arm, q):
        """Gravity torque about the J1 axis (N·m from the URDF masses, signed: + = pulls J1 in the positive direction)

        q = {joint name: rad} (joints left out = 0)"""
        W = self._world(q)
        j1 = self.joints[f"{arm}_joint_1"]
        Tj = W[j1.find("parent").get("link")] @ _origin(j1)
        ax = Tj[:3, :3] @ np.array([float(v) for v in j1.find("axis").get("xyz").split()])
        o = Tj[:3, 3]
        return float(sum(np.cross(W[n][:3, :3] @ c + W[n][:3, 3] - o, m * G) @ ax for n, m, c in self._masses(arm)))

    def j1_range(self, arm):
        lim = self.joints[f"{arm}_joint_1"].find("limit")
        return float(lim.get("lower")), float(lim.get("upper"))

    def _sweep_peak(self, arm, shape):
        lo, hi = self.j1_range(arm)
        q = {f"{arm}_joint_{k + 2}": v for k, v in enumerate(shape)}
        peak = 0.0
        for i in range(SWEEP_STEPS + 1):
            q[f"{arm}_joint_1"] = lo + (hi - lo) * i / SWEEP_STEPS
            peak = max(peak, abs(self.torque(arm, q)))
        return peak

    def reference(self, arm):
        """Peak torque during calibration = J1 sweeps its full range with joints 2-6 at 0 (100%)"""
        if arm not in self._ref:
            self._ref[arm] = self._sweep_peak(arm, [0.0] * 5)
        return self._ref[arm]

    def q_of(self, arm, values):
        return {f"{arm}_joint_{k + 1}": v for k, v in enumerate(values)}

    def percent(self, arm, values):
        """Load at this pose (values = J1..J6 rad) as % of calibration"""
        return 100.0 * abs(self.torque(arm, self.q_of(arm, values))) / self.reference(arm)

    def sweep_percent(self, arm, values):
        """If J1 turned through its full range with the arm held in this shape (joints 2-6 from values), the peak load in %"""
        return 100.0 * self._sweep_peak(arm, list(values[1:6])) / self.reference(arm)


def urdf_text(node=None, timeout=2.0):
    """The robot URDF: the /robot_description topic (if node is given and the arm system is on), otherwise run the ar_dual xacro"""
    if node is not None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        got = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(String, "/robot_description", lambda m: got.append(m.data), qos)
        import time
        end = time.monotonic() + timeout
        while not got and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        if got:
            return got[0]
    import os

    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("ar_dual"), "urdf", "humanoi_arms.urdf.xacro")
    return xacro.process_file(path).toxml()


def over(pct):
    """Over the calibration load? (compares the displayed number, rounded to an integer — 100.4% shows as 100% = not over)"""
    return round(pct) > 100


def label(pct):
    """Short text for the load level"""
    if not over(pct):
        return "✓"
    if round(pct) <= 115:
        return "⚠ over calibration"
    return "✗ very heavy"
