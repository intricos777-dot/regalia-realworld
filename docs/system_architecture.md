# System Architecture — Regalia Real-World Equivalent

## Overview

The vehicle is a **roadable VTOL aircraft** (flying car) with three flight modes:

1. **Road mode** — wings stowed, landing gear deployed, driven like a car
2. **VTOL mode** — lift fans active, landing gear retracted, hover/transition
3. **Cruise mode** — wings deployed, pusher fan provides forward thrust, lift wing provides aerodynamic lift

## Subsystems

### 1. Airframe / Structure
- **Chassis**: Monocoque tub, 4200mm long, 2600mm wide, 1150mm tall
- **Lift wing**: Full-span delta plate, 6400mm span, 70mm thick, welded through body
- **Tail**: Single vertical stabilizer, 620mm height
- **Landing gear**: Retractable tricycle, 400mm stroke

### 2. Propulsion
- **2x Mid-body lift fans**: 1700mm diameter ducted fans, vertical thrust for VTOL
- **1x Rear pusher fan**: 1400mm diameter, forward thrust for cruise
- **Motor type**: Axial-flux permanent magnet synchronous motors
- **ESC**: Independent motor controller per fan, CAN bus communication

### 3. Power
- **Battery**: Solid-state Li-ion, 120 kWh, 800V bus
- **Distribution**: Redundant power buses (A/B), pyro-disconnect for crash safety
- **Backup**: Supercapacitor buffer for peak power during transition

### 4. Avionics / Flight Control
- **Flight controller**: Triple-redundant FCC (Flight Control Computer)
- **Sensors**: IMU (6-axis), GPS, barometer, airspeed, motor RPM, current sensors
- **Control loops**: PID for attitude, rate, altitude, heading
- **Telemetry**: 915MHz radio + 4G/5G cellular backup

### 5. 3D Printing / Manufacturing
- **Material**: Carbon-fiber reinforced PETG (CF-PETG) for structural parts
- **Print orientation**: Flat on bed, supports for overhangs >45 degrees
- **Post-processing**: Acetone vapor smooth, epoxy coating for UV resistance
- **Assembly**: Bonded with aerospace-grade adhesive, bolted at load points

## Flight Modes

```
ROAD → VTOL → CRUISE → VTOL → ROAD
  │       │       │       │       │
  │       │       │       │       └── Gear deploy, fans idle
  │       │       │       └── Gear retract, lift fans spin down
  │       │       └── Wings deploy, pusher fan on
  │       └── Gear retract, lift fans spin up
  └── Gear deploy, fans idle
```

## Safety

- **Redundancy**: Triple FCC, dual power buses, independent motor controllers
- **Parachute**: Ballistic recovery system (BRS) for catastrophic failure
- **Geofencing**: GPS-based altitude and range limits
- **Fail-safe**: Auto-land on communication loss or low battery
