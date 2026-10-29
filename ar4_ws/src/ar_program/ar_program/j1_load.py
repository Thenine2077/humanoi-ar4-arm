"""ภาระน้ำหนักแขนที่ข้อ 1 (J1) — แรงบิดจากน้ำหนักทุกชิ้นที่หมุนตาม J1 รอบแกน J1 เทียบกับตอน calibrate

แขนติดด้านข้างลำตัว แกน J1 นอนขนานพื้น J1 จึงต้องยกน้ำหนักทั้งแขน (+ มือ) เหมือนยกตุ้ม ยิ่งแขนยื่นห่างแกนยิ่งหนัก
สายพาน J1 เคยรูดตอนแขนยื่นไกล ส่วนตอน calibrate (ข้ออื่นอยู่ท่าพัก แขนชี้ออกด้านข้างตามแนวแกน J1)
J1 หมุนผ่านแนวนอนได้ไม่เคยรูด -> ใช้ภาระสูงสุดตอนนั้นเป็น 100%

  ภาระ % = |แรงบิดที่ท่านั้น| / แรงบิดสูงสุดตอน J1 กวาดทั้งช่วงโดยข้อ 2-6 = 0 × 100
  เกิน 100% = หนักกว่าที่เคยพิสูจน์แล้วว่าไม่รูด

มวลมาจาก URDF (ar_arm.xacro = ค่าของ AR3 + มือ 0.4 kg) น้ำหนักจริงไม่ตรง จึงเชื่อได้แค่เปอร์เซ็นต์เทียบกัน
ไม่ใช่ N·m — ถ้าชิ้นไหนหนักกว่าในโมเดลมาก (เช่นมอเตอร์ข้อ 4-6) ตัวเลขจะเพี้ยนไปทางนั้น
ไม่ใช้ ROS (แค่อ่าน URDF) — urdf_text() เอา URDF จากหัวข้อ /robot_description หรือ xacro ของ ar_dual
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

G = np.array([0.0, 0.0, -9.81])  # ในเฟรม torso (z ขึ้น)
SWEEP_STEPS = 72  # J1 ทั้งช่วง ~340° ทีละ ~5° (ยอดคลื่นคลาดไม่ถึง 0.1%)


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
        """[(link, kg, จุดศูนย์ถ่วงในเฟรม link)] ของชิ้นที่หมุนตาม J1 — link_1..6 + มือ (ไม่รวมฐานแขน)"""
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
        """แรงบิดจากน้ำหนักรอบแกน J1 (N·m ตามมวลใน URDF มีเครื่องหมาย: + = ดึงให้ J1 หมุนทางบวก)

        q = {ชื่อข้อต่อ: rad} (ข้อที่ไม่ใส่ = 0)"""
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
        """แรงบิดสูงสุดตอน calibrate = J1 กวาดทั้งช่วง ข้อ 2-6 อยู่ 0 (100%)"""
        if arm not in self._ref:
            self._ref[arm] = self._sweep_peak(arm, [0.0] * 5)
        return self._ref[arm]

    def q_of(self, arm, values):
        return {f"{arm}_joint_{k + 1}": v for k, v in enumerate(values)}

    def percent(self, arm, values):
        """ภาระที่ท่านี้ (values = J1..J6 rad) เป็น % ของตอน calibrate"""
        return 100.0 * abs(self.torque(arm, self.q_of(arm, values))) / self.reference(arm)

    def sweep_percent(self, arm, values):
        """ถ้า J1 หมุนทั้งช่วงโดยแขนคงรูปร่างนี้ (ข้อ 2-6 ตาม values) ภาระสูงสุดกี่ %"""
        return 100.0 * self._sweep_peak(arm, list(values[1:6])) / self.reference(arm)


def urdf_text(node=None, timeout=2.0):
    """URDF ของหุ่น: หัวข้อ /robot_description (ถ้าส่ง node และระบบแขนเปิดอยู่) ไม่งั้นรัน xacro ของ ar_dual"""
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
    """เกินตอน calibrate ไหม (เทียบเลขที่แสดง ปัดเป็นจำนวนเต็มแล้ว — 100.4% แสดง 100% = ไม่เกิน)"""
    return round(pct) > 100


def label(pct):
    """ข้อความสั้นตามระดับภาระ"""
    if not over(pct):
        return "✓"
    if round(pct) <= 115:
        return "⚠ เกินตอน calibrate"
    return "✗ หนักมาก"
