# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# electronics/motor_controller.py
"""Electronic speed controller (ESC) for ducted fan motors.

Implements field-oriented control (FOC) for axial-flux PMSM motors
with CAN bus interface.
"""

import time
import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class MotorConfig:
    """Motor configuration."""
    pole_pairs: int = 8
    max_rpm: float = 8000.0
    max_current_a: float = 350.0
    max_power_kw: float = 200.0
    kv_rating: float = 120.0  # RPM per volt
    phase_resistance_ohm: float = 0.008
    phase_inductance_h: float = 0.00005


@dataclass
class MotorState:
    """Real-time motor state."""
    rpm: float = 0.0
    current_a: float = 0.0
    voltage_v: float = 0.0
    temperature_c: float = 25.0
    torque_nm: float = 0.0
    throttle_pct: float = 0.0
    fault: Optional[str] = None
    can_id: int = 0


class FOCController:
    """Field-oriented control for PMSM.

    Simplified implementation: Clarke/Park transform + PI current loops.
    """

    def __init__(self, config: MotorConfig):
        self.config = config
        # PI current controllers (simplified)
        self.kp_id = 0.5
        self.ki_id = 10.0
        self.kp_iq = 0.5
        self.ki_iq = 10.0
        self.integral_id = 0.0
        self.integral_iq = 0.0

    def clarke_transform(self, ia: float, ib: float, ic: float) -> tuple:
        """3-phase to alpha-beta."""
        ialpha = ia
        ibeta = (ia + 2 * ib) / math.sqrt(3)
        return ialpha, ibeta

    def park_transform(self, ialpha: float, ibeta: float, theta: float) -> tuple:
        """Alpha-beta to d-q."""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        id_ = ialpha * cos_t + ibeta * sin_t
        iq = -ialpha * sin_t + ibeta * cos_t
        return id_, iq

    def update(self, state: MotorState, throttle: float, dt: float = 0.001) -> tuple:
        """Update FOC loop. Returns (id_ref, iq_ref) current commands."""
        # Target torque from throttle
        torque_ref = throttle / 100.0 * self.config.max_power_kw * 1000.0 / (
            state.rpm * 2 * math.pi / 60.0 + 0.001
        )
        # iq ~ torque (simplified: iq = torque / (1.5 * pole_pairs * flux_linkage))
        flux_linkage = 0.05  # Wb (example)
        iq_ref = torque_ref / (1.5 * self.config.pole_pairs * flux_linkage)
        id_ref = 0.0  # zero d-axis current for max torque per amp

        # Clamp
        max_i = self.config.max_current_a
        iq_ref = max(-max_i, min(max_i, iq_ref))
        id_ref = max(-max_i, min(max_i, id_ref))

        return id_ref, iq_ref


class ESC:
    """Electronic speed controller with CAN interface."""

    def __init__(self, can_id: int, config: MotorConfig = None):
        self.can_id = can_id
        self.config = config or MotorConfig()
        self.foc = FOCController(self.config)
        self.state = MotorState(can_id=can_id)
        self.enabled = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        self.state.throttle_pct = 0.0

    def set_throttle(self, pct: float):
        """Set throttle 0-100%."""
        if not self.enabled:
            return
        self.state.throttle_pct = max(0.0, min(100.0, pct))

    def update(self, dt: float = 0.001):
        """Update motor state (simulated)."""
        if not self.enabled:
            return
        # Simplified motor model: rpm follows throttle with lag
        target_rpm = self.state.throttle_pct / 100.0 * self.config.max_rpm
        alpha = 1.0 - math.exp(-dt / 0.5)  # 0.5s time constant
        self.state.rpm += alpha * (target_rpm - self.state.rpm)

        # Current draw proportional to torque demand
        self.state.current_a = self.state.throttle_pct / 100.0 * self.config.max_current_a * 0.7

        # Temperature model
        power_loss = self.state.current_a ** 2 * self.config.phase_resistance_ohm * 3
        self.state.temperature_c += power_loss * 0.001 * dt
        self.state.temperature_c = min(120.0, self.state.temperature_c)

        # Fault detection
        if self.state.temperature_c > 100.0:
            self.state.fault = "OVERTEMP"
        elif self.state.current_a > self.config.max_current_a * 1.1:
            self.state.fault = "OVERCURRENT"
        else:
            self.state.fault = None

        # FOC update
        id_ref, iq_ref = self.foc.update(self.state, self.state.throttle_pct, dt)
        self.state.torque_nm = iq_ref * 1.5 * self.config.pole_pairs * 0.05


class ESCs:
    """Manager for all ESCs in the vehicle."""

    def __init__(self):
        self.controllers = {
            "lift_fan_left": ESC(can_id=0x101, config=MotorConfig(max_power_kw=150.0)),
            "lift_fan_right": ESC(can_id=0x102, config=MotorConfig(max_power_kw=150.0)),
            "pusher_fan": ESC(can_id=0x103, config=MotorConfig(max_power_kw=200.0)),
        }

    def enable_all(self):
        for esc in self.controllers.values():
            esc.enable()

    def disable_all(self):
        for esc in self.controllers.values():
            esc.disable()

    def set_throttle(self, name: str, pct: float):
        if name in self.controllers:
            self.controllers[name].set_throttle(pct)

    def update_all(self, dt: float = 0.001):
        for esc in self.controllers.values():
            esc.update(dt)

    def status(self) -> str:
        lines = ["ESC STATUS:"]
        for name, esc in self.controllers.items():
            s = esc.state
            fault_str = f"  FAULT: {s.fault}" if s.fault else ""
            lines.append(
                f"  {name:20s}  RPM={s.rpm:6.0f}  I={s.current_a:5.1f}A  "
                f"T={s.temperature_c:5.1f}C{fault_str}"
            )
        return "\n".join(lines)


if __name__ == "__main__":
    escs = ESCs()
    escs.enable_all()
    escs.set_throttle("lift_fan_left", 55)
    escs.set_throttle("lift_fan_right", 55)
    escs.set_throttle("pusher_fan", 0)

    print("--- VTOL Hover (50 steps) ---")
    for _ in range(50):
        escs.update_all(dt=0.001)
    print(escs.status())

    escs.set_throttle("pusher_fan", 70)
    print("\n--- Cruise (50 steps) ---")
    for _ in range(50):
        escs.update_all(dt=0.001)
    print(escs.status())
