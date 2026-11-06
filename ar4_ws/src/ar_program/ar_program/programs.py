"""Read/write/check the pose and program file of the two AR4 arms (shared by teach and arm_program)

Arm 1 = left (the robot's own left arm), arm 2 = right

File format (YAML):

  poses:                        # per arm — the same pose name may exist on both arms (angles belong to each arm)
    left:
      rest: [0.0, -0.1019, 0.0, 0.0, -0.9103, 0.0]   # joint_1..joint_6 in radians
    right:
      rest: [0.0, -0.1019, 0.0, 0.0, -0.9103, 0.0]
  programs:                     # step by step — each step gives a pose for each arm, or still (arm not commanded = stays where it is)
    pick2:
      - {left: act1, right: act1}
      - {left: act2, right: still, left_hand: rock}   # hand gesture (omitted = hand does not move) — moves together with the arm in the same step
      - {left: still, right: still, left_hand: paper} # hand only
      - {wait: 1.0}
    demo: [rest, look_left]     # old format (bare pose names) = left arm only

  hand_poses:                   # self-taught hand gestures (arm hand-teach): raw servo ID 1-8 degrees for the right hand set
    wave: [10, 40, -20, 30, 90, -90, 0, 0]       # same name as a built-in gesture (rock paper ...) = replaces the built-in one
  hand_poses_left: {}           # for the left hand set (a different hand, assembled mirrored — degrees are not interchangeable)

  still is not a stored position — the arm stays wherever the previous step left it. If the position were stored when the program was written, the arm would run back there
  In every step, the arms that move start together and arrive together; the next step starts when every arm has arrived
  Old-format files (poses for a single arm) can still be read — treated as left-arm poses, then rewritten in the new format on the next save

This file is user data — so the default points to src/, not install/ (install is overwritten every time
colcon build runs, and all taught poses would be lost)
"""
import math
import os

import yaml

ARMS = ("left", "right")
ARM_ALIASES = {"left": "left", "l": "left", "1": "left", "ซ้าย": "left",
               "right": "right", "r": "right", "2": "right", "ขวา": "right"}
STILL = ("still", "-")
HAND_KEYS = {"left": "left_hand", "right": "right_hand"}  # hand on the left arm / right arm
# self-taught hand gestures, per hand set (AmazingHand right hand set / left hand set — unrelated to which arm it is mounted on)
HAND_POSE_KEYS = {"right": "hand_poses", "left": "hand_poses_left"}
# hand gestures that can be commanded -> time the hand needs (seconds); the next step waits for it — names and actual gestures are in humanoi_games/hand_poses.py
# (Humanoi repo), edit both together · victory / nonono = moving gestures
HAND_GESTURES = {"rock": 0.6, "paper": 0.6, "scissors": 0.6, "point": 0.6, "like": 0.6, "windup": 0.6,
                 "fist": 0.6, "victory": 1.8, "nonono": 1.8, "relax": 0.3}
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# joint limits (rad) — same values as <ros2_control> in ar_dual/urdf/arm_control.xacro, which were converted from
# the vendor's ar.ros2_control.xacro. Every pose is checked before commanding the arm, so a typo in the file can't drive into a limit
# each arm is shifted by the zero trim in its board (ar_dual/config/zero_trims.yaml) — use limits(arm), not LIMITS directly
LIMITS = {
    "joint_1": (-2.967060, 2.967060),
    "joint_2": (-0.733038, 1.570796),
    "joint_3": (-1.553343, 0.907571),
    "joint_4": (-2.879793, 2.879793),
    "joint_5": (-1.832596, 1.832596),
    "joint_6": (-2.705260, 2.705260),
}


_TRIMS = None


def zero_trims():
    """Zero trims in each arm's board (degrees) from ar_dual/config/zero_trims.yaml — no ar_dual package = 0 for every joint"""
    global _TRIMS
    if _TRIMS is None:
        try:
            from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        except ImportError:  # running outside ROS (e.g. Windows python)
            path = None
        else:
            try:
                path = os.path.join(get_package_share_directory("ar_dual"), "config", "zero_trims.yaml")
            except PackageNotFoundError:
                path = None
        trims = {}
        if path:  # package found but file unreadable = fail loudly (limits on the wrong side could drive the arm past the switch)
            with open(path, encoding="utf-8") as f:
                trims = yaml.safe_load(f) or {}
        _TRIMS = {arm: [float(v) for v in trims.get(arm, [0] * len(JOINTS))] for arm in ARMS}
    return _TRIMS


def limits(arm=None):
    """Joint limits (rad) of that arm = LIMITS shifted by the board's zero trim (arm=None = no shift)"""
    trims = zero_trims().get(arm, [0.0] * len(JOINTS)) if arm else [0.0] * len(JOINTS)
    return {j: (LIMITS[j][0] + math.radians(t), LIMITS[j][1] + math.radians(t)) for j, t in zip(JOINTS, trims)}


def joint_names(arm, layout="dual"):
    """Joint names in /joint_states: dual = left_joint_1.. / right_joint_1.., single (old single-arm setup) = joint_1.."""
    return [f"{arm}_{j}" for j in JOINTS] if layout == "dual" else list(JOINTS)


def arm_of(token):
    arm = ARM_ALIASES.get(str(token).strip().lower())
    if arm is None:
        raise ProgramError(f"Unknown arm '{token}' — use left/1 or right/2")
    return arm


def _default_file():
    """Find the pose file in src/ on this machine (must be src, not install — install is overwritten on build)

    Order: AR_PROGRAM_FILE variable > PC (WSL) > Pi (Docker humanoi_ros, ros2_ws mounted at /humanoi_ws)
    """
    env = os.environ.get("AR_PROGRAM_FILE")
    if env:
        return env
    for path in (
        "/mnt/c/Users/User/Desktop/ar4/ar4_ws/src/ar_program/config/programs.yaml",  # PC / WSL
        "/humanoi_ws/src/ar_program/config/programs.yaml",                          # Pi / Docker
    ):
        if os.path.isdir(os.path.dirname(path)):
            return path
    return os.path.expanduser("~/ar_program/programs.yaml")


DEFAULT_FILE = _default_file()

HEADER = """\
# Poses and programs of the two AR4 arms (left = arm 1, the robot's own left; right = arm 2)
# Edit by hand, or use these commands on the Pi:
#   arm teach <left|right> <pose name>                  save the current pose of that arm
#   arm program <name> <left>/<right> ... wait=1.0 ...  e.g.  arm program pick2 act1/act1 act2/- -/act3
#   arm show <program name>                             view as a table
#
# poses    : per arm, angles joint_1..joint_6 in radians
# programs : step by step — {left: pose, right: pose|still, left_hand: hand gesture} or {wait: seconds}
#            (bare pose name = left arm only)
# hand_poses: self-taught hand gestures (arm hand-teach), servo ID 1-8 degrees — same name as a built-in gesture = replaces it
#             hand_poses = right hand set, hand_poses_left = left hand set (arm hand-side tells which one is fitted now)
# (this whole file is rewritten on every teach; comments you add will be lost, except this header)

"""


class ProgramError(ValueError):
    pass


def load(path):
    data = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    poses = data.get("poses") or {}
    # old format: {pose name: [6 values]} = poses of a single arm -> left arm
    if poses and all(isinstance(v, (list, tuple)) for v in poses.values()):
        poses = {"left": poses}
    data["poses"] = {arm: dict(poses.get(arm) or {}) for arm in ARMS}
    data["programs"] = data.get("programs") or {}
    for key in HAND_POSE_KEYS.values():
        data[key] = {str(k): [float(v) for v in vals] for k, vals in (data.get(key) or {}).items()}
    return data


def _round_in(joint, v, arm=None):
    """Round to 4 places without leaving the limits — a pose taught right at a limit (e.g. joint 5 = -1.832596) would round past it and become unplayable"""
    lo, hi = limits(arm).get(joint, (-math.inf, math.inf))
    r = round(float(v), 4)
    if r < lo:
        return math.ceil(lo * 1e4) / 1e4
    if r > hi:
        return math.floor(hi * 1e4) / 1e4
    return r


def save(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    poses = {arm: {k: [_round_in(j, v, arm) for j, v in zip(JOINTS, vals)] for k, vals in data["poses"][arm].items()}
             for arm in ARMS}
    out = {"poses": poses}
    for key in HAND_POSE_KEYS.values():
        hand_poses = {k: [round(float(v), 1) for v in vals] for k, vals in (data.get(key) or {}).items()}
        if hand_poses or key == "hand_poses":
            out[key] = hand_poses
    out["programs"] = data["programs"]
    body = yaml.safe_dump(
        out,
        allow_unicode=True,
        default_flow_style=None,  # lists of numbers / step dicts go on one line, easy to read
        sort_keys=False,
    )
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(HEADER + body)
    os.replace(tmp, path)  # write a temp file first, then swap it in, so the file isn't corrupted if the program stops midway


def check_pose(name, values, arm=None):
    """Return the list of 6 angles if valid, otherwise raise ProgramError with the reason (arm = use that arm's limits)"""
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINTS):
        raise ProgramError(f"Pose '{name}' must have {len(JOINTS)} values (joint_1..joint_6)")
    out = []
    for joint, v in zip(JOINTS, values):
        try:
            v = float(v)
        except (TypeError, ValueError):
            raise ProgramError(f"Pose '{name}' {joint} is not a number: {v!r}")
        lo, hi = limits(arm)[joint]
        if not lo <= v <= hi:
            raise ProgramError(
                f"Pose '{name}' {joint} = {v:.3f} rad ({math.degrees(v):.1f}°) "
                f"is outside the limits {lo:.3f}..{hi:.3f} rad"
            )
        out.append(v)
    return out


def step_arms(step):
    """Step in the file -> {arm: pose name|still} (old-style bare pose name = left arm), or None if it is a wait"""
    if isinstance(step, str):
        return {"left": step, "right": "still"}
    if isinstance(step, dict) and set(step) == {"wait"}:
        return None
    if isinstance(step, dict) and step and set(step) <= set(ARMS) | set(HAND_KEYS.values()):
        return {arm: str(step.get(arm, "still")) for arm in ARMS}
    raise ProgramError(f"Unknown step {step!r} (use {{left: pose, right: pose|still, left_hand: hand gesture}} "
                       "or {wait: seconds})")


def hand_names(data=None):
    """Hand gesture names that can be commanded: built-in + self-taught ones in the file (both hand sets)"""
    names = set(HAND_GESTURES)
    for key in HAND_POSE_KEYS.values():
        names |= set((data or {}).get(key) or {})
    return names


def hand_duration(name):
    """Time the hand needs for the gesture (seconds) — self-taught gestures are static"""
    return HAND_GESTURES.get(name, 0.6)


def step_hands(step):
    """Step in the file -> {arm: hand gesture} for commanded hands only (none = hand does not move)"""
    if not isinstance(step, dict):
        return {}
    return {arm: str(step[key]) for arm, key in HAND_KEYS.items() if str(step.get(key, "still")) not in STILL}


def parse_cli_step(token, hands=None):
    """One step of command text -> step in the file (hands = accepted hand gesture names; omitted = built-in ones)

    "<left pose>/<right pose>[/<left hand>[/<right hand>]]" (- or still = no command) or "wait=seconds"
    """
    if token.startswith("wait="):
        return {"wait": float(token[5:])}
    parts = [p.strip() for p in token.split("/")]
    if not 2 <= len(parts) <= 4 or not all(parts):
        raise ProgramError(f"'{token}' — write each step as <left arm>/<right arm>[/<left hand>[/<right hand>]] "
                           "(use - if it doesn't move) or wait=seconds")
    step = {"left": "still" if parts[0] in STILL else parts[0], "right": "still" if parts[1] in STILL else parts[1]}
    for arm, part in zip(ARMS, parts[2:]):
        if part in STILL:
            continue
        known = hands if hands is not None else HAND_GESTURES
        if part not in known:
            raise ProgramError(f"'{token}' — unknown hand gesture '{part}' (available: {', '.join(sorted(known))})")
        step[HAND_KEYS[arm]] = part
    return step


def trapezoid(start, target, speed, accel, min_time=0.0, dt=0.1):
    """Motion plan: accelerate → cruise at constant speed → brake; all joints start and finish together

    The joint with the farthest to go runs at speed (rad/s), accelerating/braking at accel (rad/s^2); the other joints are scaled proportionally
    Returns [(t, positions, velocities), ...] in time order; the last point = target with velocity 0
    min_time stretches the duration (used both to avoid jerks between nearby poses and so both arms in a step arrive together)

    Why not send a single point and let the controller draw its own curve: that curve peaks at 1.5x the
    average speed, while the Teensy firmware (AccelStepper, 10°/s^2 acceleration) lags the target by about v^2/2a,
    so on distant poses the lag exceeds the controller's 0.2 rad path tolerance and the motion is aborted midway
    (seen for real with the home pose at 157°: peak speed 17°/s → lag ~15°)
    """
    deltas = [t - s for s, t in zip(start, target)]
    dist = max(abs(d) for d in deltas)
    if dist < 1e-6:
        return [(max(min_time, dt), list(target), [0.0] * len(target))]
    v, a = speed, accel
    if dist >= v * v / a:            # far enough to have a constant-speed phase
        t_acc = v / a
        t_total = 2 * t_acc + (dist - v * v / a) / v
    else:                            # close — must brake before even reaching speed
        t_acc = math.sqrt(dist / a)
        v = a * t_acc
        t_total = 2 * t_acc
    stretch = max(1.0, min_time / t_total)  # stretch the duration up to min_time

    def at(t):  # distance covered (0..dist) and velocity at time t of the unstretched plan
        if t < t_acc:
            return 0.5 * a * t * t, a * t
        if t < t_total - t_acc:
            return 0.5 * a * t_acc * t_acc + v * (t - t_acc), v
        r = t_total - t
        return dist - 0.5 * a * r * r, a * r

    end = t_total * stretch
    points = []
    for k in range(1, int(math.ceil(end / dt)) + 1):
        t = min(k * dt, end)
        s, sv = at(t / stretch)
        frac, vfrac = s / dist, sv / stretch / dist
        points.append((t, [st + d * frac for st, d in zip(start, deltas)],
                       [d * vfrac for d in deltas]))
    points[-1] = (end, list(target), [0.0] * len(target))
    return points


def _pose(data, arm, name, where):
    if name not in data["poses"][arm]:
        known = ", ".join(sorted(data["poses"][arm])) or "(none yet)"
        raise ProgramError(f"{where}: the {'left' if arm == 'left' else 'right'} arm has no pose '{name}' — available: {known}")
    return check_pose(f"{arm}:{name}", data["poses"][arm][name], arm)


def resolve(data, program, arms=ARMS):
    """Turn a program into a list of checked steps: [("move", {arm: (pose name, [6 values])}, {arm: hand gesture}) | ("wait", seconds, {})]

    arms = arms that actually exist in the running system (old single-arm system = ("left",))
    The whole program is checked before anything moves — if even one step has a missing pose or exceeds a limit, nothing moves at all
    (better than running halfway and then failing)
    """
    if program.startswith("@"):  # "@pose" (left arm) or "@right:pose" = go to a single pose (used when editing poses)
        arm, _, pose = program[1:].rpartition(":")
        arm = arm_of(arm) if arm else "left"
        if arm not in arms:
            raise ProgramError(f"This system has no {arm} arm")
        return [("move", {arm: (pose, _pose(data, arm, pose, "goto"))}, {})]
    steps = data["programs"].get(program)
    if steps is None:
        known = ", ".join(sorted(data["programs"])) or "(none yet)"
        raise ProgramError(f"Program '{program}' not found — available: {known}")
    if not isinstance(steps, list) or not steps:
        raise ProgramError(f"Program '{program}' must be a non-empty list of steps")
    out = []
    for i, step in enumerate(steps, 1):
        where = f"Program '{program}' step {i}"
        try:
            per_arm = step_arms(step)
        except ProgramError as exc:
            raise ProgramError(f"{where}: {exc}")
        if per_arm is None:
            secs = float(step["wait"])
            if not 0 <= secs <= 600:
                raise ProgramError(f"{where}: wait must be between 0 and 600 seconds")
            out.append(("wait", secs, {}))
            continue
        hands = step_hands(step)
        for arm, gesture in hands.items():
            if gesture not in hand_names(data):
                raise ProgramError(f"{where}: unknown hand gesture '{gesture}' (available: {', '.join(sorted(hand_names(data)))})")
        moves = {}
        for arm, pose in per_arm.items():
            if pose in STILL:
                continue
            if arm not in arms:
                raise ProgramError(f"{where}: the running system has no {'right' if arm == 'right' else 'left'} arm")
            moves[arm] = (pose, _pose(data, arm, pose, where))
        if not moves and not hands:
            raise ProgramError(f"{where}: every arm and hand is still — to wait, use wait=seconds")
        out.append(("move", moves, hands))
    return out


def table(data, program):
    """Rows for display as a table: [(step, left arm, right arm, left hand, right hand)] — used by arm show and a future GUI"""
    rows = []
    for i, step in enumerate(data["programs"].get(program) or [], 1):
        per_arm = step_arms(step)
        if per_arm is None:
            rows.append((i, f"wait {float(step['wait']):g} s", "", "", ""))
        else:
            hands = step_hands(step)
            rows.append((i, per_arm["left"], per_arm["right"], hands.get("left", "-"), hands.get("right", "-")))
    return rows


def cli_step(row):
    """A table row turned back into the text typed in the command, e.g. act1/-/rock (trailing - fields dropped)"""
    _, left, right, lh, rh = row
    if not right and not lh:
        return left.replace("wait ", "wait=").replace(" s", "")
    parts = ["-" if p in STILL else p for p in (left, right, lh, rh)]
    while len(parts) > 2 and parts[-1] == "-":
        parts.pop()
    return "/".join(parts)
