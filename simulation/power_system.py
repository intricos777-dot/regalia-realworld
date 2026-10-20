# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# simulation/power_system.py
"""Power system simulation for the Regalia flying car.

Models battery discharge, thermal management, and power budget
for different flight phases.
"""

import math
from dataclasses import dataclass, field
from typing import List, Dict, Tuple


@dataclass
class BatteryCell:
    """Single solid-state battery cell."""
    capacity_ah: float = 50.0
    voltage_nominal: float = 3.6
    voltage_max: float = 4.2
    voltage_min: float = 2.5
    internal_resistance_ohm: float = 0.002
    temperature_c: float = 25.0
    cycle_life: int = 2000


@dataclass
class BatteryPack:
    """Full battery pack: series-parallel configuration."""
    cells_in_series: int = 222  # ~800V nominal
    parallel_strings: int = 6
    cell: BatteryCell = field(default_factory=BatteryCell)

    @property
    def total_capacity_kwh(self) -> float:
        return (self.cells_in_series * self.cell.voltage_nominal *
                self.cell.capacity_ah * self.parallel_strings / 1000.0)

    @property
    def max_power_kw(self) -> float:
        v = self.cells_in_series * self.cell.voltage_max
        i_max = self.parallel_strings * self.cell.capacity_ah * 3  # 3C discharge
        return v * i_max / 1000.0

    def voltage_at_soc(self, soc: float) -> float:
        """Estimate pack voltage at given state of charge."""
        # Simplified linear model
        v_cell = self.cell.voltage_min + (self.cell.voltage_max - self.cell.voltage_min) * soc / 100.0
        return self.cells_in_series * v_cell

    def internal_resistance(self) -> float:
        return self.cells_in_series * self.cell.internal_resistance_ohm / self.parallel_strings


@dataclass
class PowerPhase:
    """A flight phase with power profile."""
    name: str
    duration_s: float
    lift_fan_pct: float
    pusher_fan_pct: float
    avionics_kw: float = 2.0
    thermal_kw: float = 5.0


class PowerBudget:
    """Calculate power budget for a flight plan."""

    def __init__(self, pack: BatteryPack = None):
        self.pack = pack or BatteryPack()
        self.max_motor_power_kw = 500.0  # total motor power

    def phase_power(self, phase: PowerPhase) -> float:
        """Total power draw during a phase."""
        motor = self.max_motor_power_kw * (
            phase.lift_fan_pct + phase.pusher_fan_pct
        ) / 200.0  # two lift fans + one pusher
        return motor + phase.avionics_kw + phase.thermal_kw

    def phase_energy(self, phase: PowerPhase) -> float:
        """Energy consumed during a phase (kWh)."""
        return self.phase_power(phase) * phase.duration_s / 3600.0

    def flight_plan(self, phases: List[PowerPhase]) -> Dict:
        """Analyze a full flight plan."""
        total_energy = 0.0
        total_time = 0.0
        results = []

        for phase in phases:
            power = self.phase_power(phase)
            energy = self.phase_energy(phase)
            total_energy += energy
            total_time += phase.duration_s
            results.append({
                "phase": phase.name,
                "duration_s": phase.duration_s,
                "power_kw": round(power, 1),
                "energy_kwh": round(energy, 2),
            })

        capacity = self.pack.total_capacity_kwh
        remaining = capacity - total_energy
        remaining_pct = remaining / capacity * 100.0

        return {
            "phases": results,
            "total_energy_kwh": round(total_energy, 2),
            "total_time_s": total_time,
            "total_time_min": round(total_time / 60, 1),
            "battery_capacity_kwh": round(capacity, 1),
            "remaining_kwh": round(remaining, 2),
            "remaining_pct": round(remaining_pct, 1),
            "feasible": remaining > 0,
        }


if __name__ == "__main__":
    pack = BatteryPack()
    print(f"Battery: {pack.total_capacity_kwh:.1f} kWh, {pack.max_power_kw:.0f} kW max")

    budget = PowerBudget(pack)

    # Example flight plan: takeoff → climb → cruise → descent → landing
    phases = [
        PowerPhase("takeoff", 60, 80, 0, thermal_kw=8),
        PowerPhase("climb", 120, 70, 20, thermal_kw=6),
        PowerPhase("cruise", 600, 15, 65, thermal_kw=4),
        PowerPhase("descent", 120, 30, 10, thermal_kw=4),
        PowerPhase("landing", 60, 75, 0, thermal_kw=6),
    ]

    plan = budget.flight_plan(phases)
    print(f"\nFlight plan ({plan['total_time_min']:.0f} min):")
    for p in plan["phases"]:
        print(f"  {p['phase']:10s}  {p['duration_s']:5.0f}s  {p['power_kw']:6.1f} kW  {p['energy_kwh']:5.2f} kWh")
    print(f"\nTotal energy: {plan['total_energy_kwh']:.1f} kWh")
    print(f"Remaining:    {plan['remaining_kwh']:.1f} kWh ({plan['remaining_pct']:.1f}%)")
    print(f"Feasible:     {plan['feasible']}")
