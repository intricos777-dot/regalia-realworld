# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# scripts/render_preview.py
"""Render the exported .STL to a few 3-D PNG views so I can eyeball the model."""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
STL = os.path.join(OUT, "regalia_realworld.STL")


def read_stl(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    if raw[:6] in (b"solid", b"Solid"):
        return _read_ascii(raw)
    return _read_binary(raw)


def _read_ascii(raw):
    lines = raw.decode().splitlines()
    verts, tris = [], []
    for ln in lines:
        parts = ln.split()
        if len(parts) >= 4 and parts[0] == "vertex":
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif len(parts) >= 4 and parts[0] == "facet" and parts[1] == "normal":
            # read next 3 vertex lines
            for _ in range(4):
                ln = next(lines_iter)
    # simpler
    verts, tris = [], []
    it = iter(lines)
    for ln in it:
        p = ln.split()
        if not p:
            continue
        if p[0] == "vertex":
            verts.append([float(p[1]), float(p[2]), float(p[3])])
        elif p[0] == "endfacet":
            pass
    tris = []
    # build triangles from groups of 3 vertices between facet lines
    idx = []
    for i, ln in enumerate(lines):
        p = ln.split()
        if p and p[0] == "vertex":
            idx.append(i)
    for k in range(0, len(idx) - 2, 3):
        a, b, c = idx[k], idx[k + 1], idx[k + 2]
        tris.append([a, b, c])
    return np.array(verts), np.array(tris)


def _read_binary(raw):
    # 80-byte header, uint32 tri count, then per tri:
    # 12B normal + 36B vertices (3x3 floats) + 2B attr = 50B
    import struct
    n = struct.unpack("<I", raw[80:84])[0]
    off = 84
    verts = []
    for _ in range(n):
        off += 12  # skip normal
        for _ in range(3):
            v = struct.unpack("<3f", raw[off:off + 12])
            off += 12
            verts.append(v)
        off += 2  # attribute byte count
    return np.array(verts), np.arange(len(verts)).reshape(-1, 3)


verts, tris = read_stl(STL)
print("verts", verts.shape, "tris", tris.shape)
np.save(os.path.join(OUT, "verts.npy"), verts)
np.save(os.path.join(OUT, "tris.npy"), tris)


def center_scale(v):
    m = v.mean(axis=0)
    return v - m, v.std() or 1.0


def render(ax, elev, azim, title):
    poly = verts[tris]
    pc = Poly3DCollection(poly, facecolor="#9fc3ff", edgecolor="#111", linewidth=0.15, alpha=0.95)
    ax.add_collection3d(pc)
    mx = np.abs(verts).max()
    ax.set_xlim(-mx, mx); ax.set_ylim(-mx, mx); ax.set_zlim(-mx, mx)
    ax.set_box_aspect((2, 2, 2))
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title)
    ax.axis("off")
    plt.savefig(os.path.join(OUT, "preview_%s.png" % title), dpi=110, bbox_inches="tight")
    plt.close()


mx = np.abs(verts).max()
fig = plt.figure(figsize=(14, 5))
render(fig.add_subplot(1, 3, 1, projection="3d"), 25, -55, "front")
render(fig.add_subplot(1, 3, 2, projection="3d"), 25, 55, "reverse")
render(fig.add_subplot(1, 3, 3, projection="3d"), 40, 30, "isometric")
print("saved")
