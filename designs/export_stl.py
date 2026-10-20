# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# designs/export_stl.py
"""Export utilities for the Regalia flying car parametric model.

Provides mesh (STL/AMF), B-Rep (STEP), and 2-D projection (SVG) export from a
cadquery model.  STL/STEP are the formats referenced by the README pipeline.

Coordinate system: +X forward, +Y starboard, +Z up.  Units are millimetres.

Run:  python designs/export_stl.py
"""

import os
import sys

import cadquery as cq

# Allow running as `python designs/export_stl.py` from the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from designs.parametric_model import build

DEFAULT_OUTDIR = os.path.join(os.path.dirname(__file__), "..", "out")


def export_stl(model, path, tolerance=0.1, angular_tolerance=0.2):
    """Export a triangulated mesh as binary STL."""
    cq.exporters.export(
        model,
        path,
        exportType="STL",
        tolerance=tolerance,
        angularTolerance=angular_tolerance,
    )
    return path


def export_amf(model, path):
    """Export a triangulated mesh as additive-manufacturing (AMF) XML."""
    cq.exporters.export(model, path, exportType="AMF")
    return path


def export_step(model, path):
    """Export a B-Rep model as STEP (AP214/AP203)."""
    cq.exporters.export(model, path, exportType="STEP")
    return path


def export_svg(model, path):
    """Export a 2-D engineering projection as SVG."""
    cq.exporters.export(model, path, exportType="SVG")
    return path


def bounding_box(model):
    """Return the model bounding box as a dict of (min, max) tuples in mm."""
    bb = model.val().BoundingBox()
    return {
        "x": (round(bb.xmin, 2), round(bb.xmax, 2)),
        "y": (round(bb.ymin, 2), round(bb.ymax, 2)),
        "z": (round(bb.zmin, 2), round(bb.zmax, 2)),
    }


def export_all(model, outdir=None):
    """Export STL, STEP, and SVG.  Returns a dict of format -> path."""
    outdir = outdir or DEFAULT_OUTDIR
    os.makedirs(outdir, exist_ok=True)
    paths = {
        "stl": os.path.join(outdir, "regalia_realworld.STL"),
        "step": os.path.join(outdir, "regalia_realworld.STEP"),
        "amf": os.path.join(outdir, "regalia_realworld.amf"),
        "svg": os.path.join(outdir, "regalia_realworld.svg"),
    }
    export_stl(model, paths["stl"])
    export_step(model, paths["step"])
    export_amf(model, paths["amf"])
    export_svg(model, paths["svg"])
    return paths


def main():
    model = build()
    paths = export_all(model)
    for fmt, path in paths.items():
        size = os.path.getsize(path)
        print(f"{fmt.upper():5s}: {path}  ({size:,} bytes)")
    print("BBox (mm):", bounding_box(model))


if __name__ == "__main__":
    main()
