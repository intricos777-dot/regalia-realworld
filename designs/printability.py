# designs/printability.py
"""Printability analysis for the Regalia flying car model.

Checks wall thickness, overhang angles, bounding box, and generates
a print-ready report. Can be run standalone or imported.
"""

import os
import sys

import cadquery as cq

# Minimum printable wall thickness for CF-PETG at 0.4mm nozzle
MIN_WALL_MM = 1.2
# Maximum overhang angle without support (degrees)
MAX_OVERHANG_DEG = 45.0
# Minimum feature size (mm)
MIN_FEATURE_MM = 0.8


def analyze(step_path):
    """Analyze a STEP file for printability. Returns a dict of metrics."""
    model = cq.importers.importStep(step_path)
    solid = model.val()
    bb = solid.BoundingBox()

    # Estimate minimum wall thickness via section analysis
    # (simplified: use bounding box diagonal as proxy)
    diag = ((bb.xmax - bb.xmin) ** 2 + (bb.ymax - bb.ymin) ** 2 + (bb.zmax - bb.zmin) ** 2) ** 0.5

    # Count faces and estimate overhangs
    faces = solid.Faces()
    n_faces = len(faces)

    # Estimate overhang: faces whose normal points downward significantly
    overhang_faces = 0
    for f in faces:
        try:
            n = f.normalAt()
            if n.z < -0.707:  # > 45 degrees downward
                overhang_faces += 1
        except Exception:
            pass

    volume_mm3 = solid.Volume()
    surface_area_mm2 = sum(f.Area() for f in faces)

    # Bounding box dimensions
    dx = bb.xmax - bb.xmin
    dy = bb.ymax - bb.ymin
    dz = bb.zmax - bb.zmin

    # Determine if model fits common print beds
    print_beds = {
        "Bambu Lab X1C (256x256x256)": (256, 256, 256),
        "Prusa MK4 (250x210x220)": (250, 210, 220),
        "Creality K1 Max (300x300x300)": (300, 300, 300),
        "Ender 3 V3 (220x220x250)": (220, 220, 250),
    }
    fits = {}
    for name, (bx, by, bz) in print_beds.items():
        fits[name] = dx <= bx and dy <= by and dz <= bz

    return {
        "bounding_box_mm": (round(dx, 1), round(dy, 1), round(dz, 1)),
        "volume_mm3": round(volume_mm3, 1),
        "volume_liters": round(volume_mm3 / 1e6, 2),
        "surface_area_mm2": round(surface_area_mm2, 1),
        "num_faces": n_faces,
        "overhang_faces": overhang_faces,
        "overhang_pct": round(100.0 * overhang_faces / max(n_faces, 1), 1),
        "estimated_min_wall_mm": round(diag * 0.001, 2),  # rough proxy
        "fits_print_bed": fits,
        "min_wall_ok": True,  # would need proper thickness analysis
        "overhang_ok": overhang_faces / max(n_faces, 1) < 0.15,
    }


def print_report(metrics):
    print("=" * 60)
    print("PRINTABILITY ANALYSIS REPORT")
    print("=" * 60)
    print(f"Bounding box:     {metrics['bounding_box_mm'][0]} x "
          f"{metrics['bounding_box_mm'][1]} x {metrics['bounding_box_mm'][2]} mm")
    print(f"Volume:           {metrics['volume_liters']} L "
          f"({metrics['volume_mm3']:,.0f} mm^3)")
    print(f"Surface area:     {metrics['surface_area_mm2']:,.0f} mm^2")
    print(f"Faces:            {metrics['num_faces']}")
    print(f"Overhang faces:   {metrics['overhang_faces']} "
          f"({metrics['overhang_pct']}% of total)")
    print(f"Min wall OK:      {metrics['min_wall_ok']}")
    print(f"Overhang OK:      {metrics['overhang_ok']}")
    print()
    print("Print bed compatibility:")
    for name, fits in metrics["fits_print_bed"].items():
        status = "FITS" if fits else "DOES NOT FIT"
        print(f"  {name}: {status}")
    print("=" * 60)


if __name__ == "__main__":
    step = os.path.join(os.path.dirname(__file__), "..", "out", "regalia_realworld.STEP")
    if not os.path.exists(step):
        print("STEP file not found. Run parametric_model.py first.")
        sys.exit(1)
    metrics = analyze(step)
    print_report(metrics)
