# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# designs/parametric_model.py
"""Parametric 3D model of a real-world flying car (Regalia, FFXV equivalent).

Generates a printable model from engineering parameters and exports STL + STEP.

CAD kernel: cadquery 2.8 (OCP / OpenCASCADE).
Coordinate system: +X forward, +Y starboard, +Z up.
Run:  python3 parametric_model.py
"""

import os

import cadquery as cq

# Design parameters (mm)
P = dict(
    wheelbase=2900.0,
    track=1900.0,
    body_length=4200.0,
    body_height=1150.0,
    body_width=2000.0,
    wing_span=6400.0,
    tail_height=620.0,
    fan_diameter=1700.0,
    rear_fan_diameter=1400.0,
    wing_root_chord=1100.0,
    wing_tip_chord=420.0,
    wing_thickness=70.0,
    landing_gear_stroke=400.0,
    scale=1.0,
)


def _fillet_bottom(wp, r=70.0):
    try:
        return wp.faces("<Z").edges().fillet(r)
    except Exception:
        return wp


def box(x, y, z, cx=0.0, cy=0.0, cz=0.0):
    """Place a centered box on the XY workplane at (cx,cy,cz)."""
    out = cq.Workplane("XY").box(x, y, z, centered=True)
    return out.translate((cx, cy, cz))


def chassis():
    """Main body tub with a rounded underside."""
    p = P
    w = cq.Workplane("XY").box(
        p["body_length"] - 400, p["track"] + 700, p["body_height"], centered=True
    )
    w = _fillet_bottom(w, 70.0)
    lip = cq.Workplane("XY").box(p["body_length"], p["track"] + 400, 130, centered=True)
    return w.union(lip)


def cabin():
    """Front cockpit cut-out with a small glazed inset."""
    w = cq.Workplane("XY").box(1800, 860, 460, centered=True)
    gl = cq.Workplane("XY").box(1660, 700, 14, centered=True)
    return w, gl


def wing_slab():
    """Full-span delta-like lift plate.  A single thin slab from the port to
    starboard extreme (the flying-wing lift surface), welded through the
    body so the model reads as one flying architecture."""
    p = P
    return cq.Workplane("XY").box(p["body_length"], p["wing_span"], p["wing_thickness"], centered=True)


def duct_fan(d):
    """Vertical thrust duct: cowl annulus + solid hub, axis along Z."""
    cowl = cq.Workplane("XY").circle(d).circle(d - 30).extrude(760)
    hub = cq.Workplane("XY").circle(d * 0.16).circle(d * 0.16 - 10).extrude(260)
    return cowl, hub


def tail_fin():
    return (
        cq.Workplane("XY")
        .workplane()
        .center(0, P["track"] / 2 + 30)
        .box(340, P["tail_height"], 135, centered=True)
    )


def landing_gear(track, stroke):
    """Retractable tricycle gear: nose skid, two main legs + wheels."""
    bh = P["body_height"] / 2.0
    z = -bh - stroke - 60
    main = cq.Workplane("XY").box(track * 0.95, stroke + 40, 180, centered=True).translate((0, 0, z))
    boot = cq.Workplane("XY").box(track * 0.95, stroke + 60, 140, centered=True).translate((0, 0, z - 10))
    nose = cq.Workplane("XY").box(240, 90, stroke + 140, centered=True).translate((600, 0, z - 5))
    tyre = cq.Workplane("XY").circle(270).circle(210).extrude(70)
    left_w = tyre.translate((0, track / 2 - 60, z - 10))
    right_w = tyre.translate((0, -(track / 2 - 60), z - 10))
    return main, boot, nose, left_w, right_w


def build():
    p = P
    parts = [chassis()]

    # Cabin + glazing
    ch, gl = cabin()
    parts.append(ch)
    parts.append(gl)

    # Headlamp pods
    for s in (+1, -1):
        lp = cq.Workplane("XY").circle(190).circle(150).extrude(70)
        parts.append(lp.translate((0, s * 500, 0)))

    # Transverse lifting wing (full-span delta plate)
    parts.append(wing_slab())

    # Ducted fans: two mid lift fans + rear pusher fan (vertical, along Z)
    for s in (+1, -1):
        cowl, hub = duct_fan(p["fan_diameter"])
        parts.append(cowl.translate((0, s * 520, -130)))
        parts.append(hub.translate((0, s * 520, -130)))
    rc, rh = duct_fan(p["rear_fan_diameter"])
    parts.append(rc.translate((1700, 0, -170)))
    parts.append(rh.translate((1700, 0, -170)))

    # Tail fins (one, on the starboard side)
    parts.append(tail_fin().translate((0, 0, 0)))

    # Retractable landing gear
    m, b, n, wl, wr = landing_gear(p["track"], p["landing_gear_stroke"])
    parts += [m, b, n, wl, wr]

    model = parts[0]
    for part in parts[1:]:
        model = model.union(part)
    return model


def export(model, outdir=None):
    outdir = outdir or os.path.join(os.path.dirname(__file__), "..", "out")
    os.makedirs(outdir, exist_ok=True)
    stl = os.path.join(outdir, "regalia_realworld.STL")
    step = os.path.join(outdir, "regalia_realworld.STEP")
    cq.exporters.export(model, stl)
    cq.exporters.export(model, step, exportType="STEP")
    return stl, step


def main():
    model = build()
    stl, step = export(model)
    print("STL :", stl)
    print("STEP:", step)


if __name__ == "__main__":
    main()
