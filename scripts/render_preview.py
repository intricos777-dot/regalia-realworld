# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# scripts/render_preview.py
"""Render the exported .STL to a few 3-D PNG views so the model can be eyeballed.

Robust STL loader (binary or ASCII), guarded so importing the module is safe.
Writes three view images plus a combined sheet into the out/ directory.

Run:  python scripts/render_preview.py [path/to/model.STL]
"""

import argparse
import os
import struct
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
DEFAULT_STL = os.path.join(OUT, "regalia_realworld.STL")

VIEWS = [
    (25, -55, "front"),
    (25, 55, "reverse"),
    (40, 30, "isometric"),
]


def _looks_binary(raw):
    """Binary STL if the file length matches the 84-byte header + 50*n rule.

    Binary STL headers frequently begin with the bytes ``solid``, so the
    header alone is not a reliable discriminator.
    """
    if len(raw) < 84:
        return False
    n = struct.unpack("<I", raw[80:84])[0]
    expected = 84 + 50 * n
    # Allow a few bytes of trailing padding from some slicers.
    return abs(len(raw) - expected) <= 4


def _read_ascii(raw):
    lines = raw.decode("ascii", errors="ignore").splitlines()
    verts = []
    tris = []
    pending = []
    for ln in lines:
        parts = ln.split()
        if not parts:
            continue
        if parts[0] == "vertex" and len(parts) >= 4:
            pending.append([float(parts[1]), float(parts[2]), float(parts[3])])
            if len(pending) == 3:
                tris.append([len(verts), len(verts) + 1, len(verts) + 2])
                verts.extend(pending)
                pending = []
    if not verts:
        raise ValueError("ASCII STL contained no triangles")
    return np.array(verts, dtype=np.float64), np.array(tris, dtype=np.int64)


def _read_binary(raw):
    n = struct.unpack("<I", raw[80:84])[0]
    off = 84
    verts = np.empty((n * 3, 3), dtype=np.float64)
    k = 0
    for _ in range(n):
        off += 12  # skip normal
        for _ in range(3):
            verts[k] = struct.unpack("<3f", raw[off:off + 12])
            off += 12
            k += 1
        off += 2  # attribute byte count
    tris = np.arange(len(verts)).reshape(-1, 3)
    return verts, tris


def read_stl(path):
    """Read an STL file (binary or ASCII) -> (verts, tris)."""
    with open(path, "rb") as fh:
        raw = fh.read()
    if _looks_binary(raw):
        return _read_binary(raw)
    if raw[:6].lower() == b"solid":
        return _read_ascii(raw)
    # Last resort: try binary anyway.
    try:
        return _read_binary(raw)
    except Exception:
        return _read_ascii(raw)


def render(ax, verts, tris, elev, azim, title):
    """Draw one view onto a 3-D axis."""
    poly = verts[tris]
    pc = Poly3DCollection(
        poly, facecolor="#9fc3ff", edgecolor="#111111", linewidth=0.15, alpha=0.95
    )
    ax.add_collection3d(pc)
    mx = float(np.abs(verts).max()) or 1.0
    ax.set_xlim(-mx, mx)
    ax.set_ylim(-mx, mx)
    ax.set_zlim(-mx, mx)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title)
    ax.set_axis_off()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stl", nargs="?", default=DEFAULT_STL, help="Path to the STL file")
    parser.add_argument("--outdir", default=OUT, help="Directory for output images")
    args = parser.parse_args(argv)

    os.makedirs(args.outdir, exist_ok=True)
    verts, tris = read_stl(args.stl)
    print(f"loaded {args.stl}: {verts.shape[0]} verts, {tris.shape[0]} tris")
    np.save(os.path.join(args.outdir, "verts.npy"), verts)
    np.save(os.path.join(args.outdir, "tris.npy"), tris)

    fig = plt.figure(figsize=(18, 6))
    for i, (elev, azim, title) in enumerate(VIEWS, start=1):
        ax = fig.add_subplot(1, 3, i, projection="3d")
        render(ax, verts, tris, elev, azim, title)
        fig.savefig(
            os.path.join(args.outdir, f"preview_{title}.png"),
            dpi=110,
            bbox_inches="tight",
        )
    sheet = os.path.join(args.outdir, "preview_sheet.png")
    fig.savefig(sheet, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print("saved:", ", ".join([f"preview_{t}.png" for _, _, t in VIEWS] + ["preview_sheet.png"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
