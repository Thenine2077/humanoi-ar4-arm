"""Flatten an onshape-to-robot MJCF (AmazingHand) at its reference pose into a few merged binary STLs.

usage: python -I mjcf_flatten.py <mjcf_dir> <out_dir>
Every visual mesh geom is placed with the composed body/geom pos+quat (joints at 0 = CAD pose).
Parts are grouped by colour (dark / light); small hardware (screws, pins, washers, rods) is skipped.
"""
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

CELL = float(os.environ.get("CELL", "0.0012"))  # m, grid size for decimation
SKIP = ("screw", "pin_", "washer", "m2_rod", "spacer", "std00", "bushing")


def quat_mat(w, x, y, z):
    n = (w * w + x * x + y * y + z * z) ** 0.5
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def frame(el):
    T = np.eye(4)
    T[:3, 3] = [float(v) for v in el.get("pos", "0 0 0").split()]
    T[:3, :3] = quat_mat(*[float(v) for v in el.get("quat", "1 0 0 0").split()])
    return T


def read_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    if 84 + 50 * n == len(d):
        a = np.frombuffer(d[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
        return a["v"].reshape(-1, 3, 3).astype(np.float64)
    tris, cur = [], []
    for line in d.decode(errors="replace").splitlines():
        p = line.split()
        if p[:1] == ["vertex"]:
            cur.append([float(v) for v in p[1:4]])
            if len(cur) == 3:
                tris.append(cur)
                cur = []
    return np.array(tris, dtype=np.float64)


def cluster(tris, cell):
    """vertex-clustering decimation: snap vertices to a grid of size cell, drop collapsed/duplicate triangles"""
    v = tris.reshape(-1, 3)
    key = np.floor(v / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    rep = np.zeros((len(uniq), 3))
    np.add.at(rep, inv, v)
    rep /= np.bincount(inv, minlength=len(uniq))[:, None]
    idx = inv.reshape(-1, 3)
    ok = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    idx = idx[ok]
    _, first = np.unique(np.sort(idx, axis=1), axis=0, return_index=True)
    idx = idx[np.sort(first)]
    return rep[idx]


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 0)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"] = nrm
    rec["v"] = tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"AmazingHand right, flattened from Pollen Robotics MJCF (Apache-2.0)".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def main():
    src, out = sys.argv[1], sys.argv[2]
    root = ET.parse(os.path.join(src, "robot.xml")).getroot()
    meshdir = os.path.join(src, root.find("compiler").get("meshdir", "."))
    files = {}
    for m in root.iter("mesh"):
        files[m.get("name") or os.path.splitext(m.get("file"))[0]] = os.path.join(meshdir, m.get("file"))
    colour = {m.get("name"): [float(v) for v in m.get("rgba").split()] for m in root.iter("material")}
    groups = {"dark": [], "light": []}
    cache = {}
    used = {}

    def walk(body, T):
        T = T @ frame(body)
        for g in body.findall("geom"):
            name = g.get("mesh")
            if not name or any(s in name for s in SKIP):
                continue
            if name not in cache:
                cache[name] = read_stl(files[name])
            G = T @ frame(g)
            v = cache[name] @ G[:3, :3].T + G[:3, 3]
            rgb = colour.get(g.get("material"), [0.8, 0.8, 0.8, 1])[:3]
            groups["dark" if sum(rgb) / 3 < 0.5 else "light"].append(v)
            used[name] = used.get(name, 0) + 1
        for b in body.findall("body"):
            walk(b, T)

    for b in root.find("worldbody").findall("body"):
        walk(b, np.eye(4))
    os.makedirs(out, exist_ok=True)
    allv = []
    for k, parts in groups.items():
        tris = cluster(np.concatenate(parts), CELL)
        write_stl(os.path.join(out, f"ah_right_{k}.stl"), tris)
        allv.append(tris.reshape(-1, 3))
        print(f"{k}: {len(parts)} parts, {len(tris)} triangles")
    allv = np.concatenate(allv)
    print("bbox min", np.round(allv.min(0), 4), "max", np.round(allv.max(0), 4))
    wi = cache["r_wrist_interface"]
    print("parts used:", dict(sorted(used.items())))


if __name__ == "__main__":
    main()
