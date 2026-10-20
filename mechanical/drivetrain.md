<!-- Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work -->
# Drivetrain — Regalia Real-World Equivalent

## Overview

The vehicle uses a **distributed electric propulsion** system with no mechanical drivetrain between motors. Each ducted fan has its own axial-flux PMSM motor and ESC.

## Motor Configuration

| Motor | Type | Power | RPM | Thrust |
|-------|------|-------|-----|--------|
| Lift Fan L | Axial-flux PMSM | 150 kW | 8000 | 5000 N |
| Lift Fan R | Axial-flux PMSM | 150 kW | 8000 | 5000 N |
| Pusher Fan | Axial-flux PMSM | 200 kW | 8000 | 4000 N |

## Road Mode

In road mode, the vehicle is driven by **in-wheel motors** on the rear axle (2x 50 kW). The ducted fans are stowed and idle.

## Power Flow

```
Battery (800V) → PDU → Bus A/B → ESC → Motor → Fan → Thrust
                                    ↓
                              Avionics (12V/48V)
```

## Thermal Management

- Liquid cooling loop for motors and ESCs
- Air cooling for battery pack
- Phase-change material (PCM) buffer for peak thermal loads
