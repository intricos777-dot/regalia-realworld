# Regalia Real-World Equivalent — Flying Car

A parametric, 3D-printable flying car inspired by the **Regalia** from *Final Fantasy XV* (also appearing as a mount in *Final Fantasy XIV*). This repository contains the complete engineering architecture: parametric CAD model, printability analysis, flight control backend, power distribution, telemetry, and simulation.

## Quick Start

```bash
# Install dependencies
python3 -m venv .venv && source .venv/bin/activate
uv pip install cadquery numpy matplotlib

# Generate the 3D model (STL + STEP)
python3 designs/parametric_model.py

# Render preview images
python3 scripts/render_preview.py

# Run flight simulation
python3 simulation/flight_sim.py
```

## Repository Structure

```
regalia-realworld/
├── designs/                  # 3D-printable design pipeline
│   ├── parametric_model.py   # Parametric CAD model (cadquery)
│   ├── printability.py       # Printability analysis
│   └── export_stl.py         # STL/STEP export utilities
├── electronics/              # Backend flight/avionics code
│   ├── flight_controller.py  # PID flight control loop
│   ├── power_distribution.py # Power bus management
│   ├── motor_controller.py   # ESC/motor control
│   └── telemetry.py          # Sensor fusion + telemetry
├── simulation/               # Physics simulation
│   ├── flight_dynamics.py    # 6-DOF flight dynamics
│   ├── power_system.py       # Battery + power budget
│   └── flight_sim.py         # Integrated flight simulator
├── mechanical/               # Mechanical design docs
│   ├── drivetrain.md
│   └── bill_of_materials.md
├── docs/
│   └── system_architecture.md
├── scripts/
│   └── render_preview.py     # 3D preview renderer
├── out/                      # Generated STL/STEP/PNG files
├── .github/workflows/
│   └── test.yml              # CI: lint + generate + test
└── LICENSE
```

## Design Philosophy

The Regalia is a luxury convertible that transforms into a flying vehicle. This real-world equivalent uses:

- **Ducted fan VTOL** — two mid-body lift fans + one rear pusher fan
- **Transverse lift wing** — full-span delta plate for forward-flight lift
- **Retractable tricycle landing gear** — roadable configuration
- **Distributed electric propulsion** — independent motor controllers per fan

## License

MIT — see [LICENSE](LICENSE).
