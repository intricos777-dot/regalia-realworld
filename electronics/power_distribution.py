# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# electronics/power_distribution.py
"""Power distribution system for the Regalia flying car.

Manages the 800V DC bus, battery monitoring, and redundant power feeds
to all motor controllers and avionics.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import time


@dataclass
class BatteryPack:
    """Solid-state battery pack state."""
    capacity_kwh: float = 120.0
    voltage_nominal: float = 800.0
    voltage_current: float = 800.0
    current_a: float = 0.0
    soc_percent: float = 100.0  # state of charge
    temperature_c: float = 25.0
    cycle_count: int = 0
    healthy: bool = True

    def power_kw(self) -> float:
        return self.voltage_current * self.current_a / 1000.0

    def range_km(self, avg_consumption_kwh_km: float = 0.25) -> float:
        """Estimate remaining range at given consumption."""
        return (self.soc_percent / 100.0 * self.capacity_kwh) / avg_consumption_kwh_km


@dataclass
class PowerBus:
    """A power distribution bus."""
    name: str
    voltage: float = 800.0
    current_a: float = 0.0
    active: bool = True
    overcurrent_limit_a: float = 500.0


@dataclass
class MotorLoad:
    """A motor load on the bus."""
    name: str
    power_kw: float = 0.0
    current_a: float = 0.0


class PowerDistribution:
    """Main power distribution unit (PDU)."""

    def __init__(self):
        self.battery = BatteryPack()
        self.bus_a = PowerBus("A")
        self.bus_b = PowerBus("B")
        self.loads: Dict[str, MotorLoad] = {}
        self.total_power_kw = 0.0
        self.fault: Optional[str] = None

        # Register motor loads
        self._register_load("lift_fan_left")
        self._register_load("lift_fan_right")
        self._register_load("pusher_fan")
        self._register_load("avionics")
        self._register_load("landing_gear")
        self._register_load("thermal_mgmt")

    def _register_load(self, name: str):
        self.loads[name] = MotorLoad(name=name)

    def set_load(self, name: str, throttle_pct: float):
        """Set motor load as percentage of max power."""
        max_power = {
            "lift_fan_left": 150.0,
            "lift_fan_right": 150.0,
            "pusher_fan": 200.0,
            "avionics": 2.0,
            "landing_gear": 5.0,
            "thermal_mgmt": 8.0,
        }
        if name in self.loads and name in max_power:
            self.loads[name].power_kw = max_power[name] * throttle_pct / 100.0

    def update(self, dt: float = 1.0):
        """Update power distribution state."""
        total = sum(l.power_kw for l in self.loads.values())
        self.total_power_kw = total

        # Update battery
        self.battery.current_a = total * 1000.0 / self.battery.voltage_current
        # Simple SOC drain model
        energy_kwh = total * dt / 3600.0
        self.battery.soc_percent -= energy_kwh / self.battery.capacity_kwh * 100.0
        self.battery.soc_percent = max(0.0, self.battery.soc_percent)

        # Check overcurrent
        for bus in (self.bus_a, self.bus_b):
            bus.current_a = self.battery.current_a / 2.0  # split across buses
            if bus.current_a > bus.overcurrent_limit_a:
                bus.active = False
                self.fault = f"Bus {bus.name} overcurrent: {bus.current_a:.0f}A"

    def status(self) -> str:
        lines = [
            "=" * 50,
            "POWER DISTRIBUTION STATUS",
            "=" * 50,
            f"Battery:  {self.battery.voltage_current:.0f}V  "
            f"{self.battery.soc_percent:.1f}%  "
            f"{self.battery.power_kw():.1f}kW  "
            f"{self.battery.temperature_c:.1f}C",
            f"Total:    {self.total_power_kw:.1f} kW",
            f"Range:    {self.battery.range_km():.0f} km",
            "",
            "Bus A:",
            f"  {self.bus_a.voltage:.0f}V  {self.bus_a.current_a:.0f}A  "
            f"{'ACTIVE' if self.bus_a.active else 'OFFLINE'}",
            "Bus B:",
            f"  {self.bus_b.voltage:.0f}V  {self.bus_b.current_a:.0f}A  "
            f"{'ACTIVE' if self.bus_b.active else 'OFFLINE'}",
            "",
            "Loads:",
        ]
        for name, load in self.loads.items():
            lines.append(f"  {name:20s} {load.power_kw:8.1f} kW")
        if self.fault:
            lines.append(f"\nFAULT: {self.fault}")
        lines.append("=" * 50)
        return "\n".join(lines)


if __name__ == "__main__":
    pdu = PowerDistribution()
    # Simulate VTOL hover
    pdu.set_load("lift_fan_left", 55)
    pdu.set_load("lift_fan_right", 55)
    pdu.set_load("pusher_fan", 0)
    pdu.set_load("avionics", 100)
    pdu.set_load("landing_gear", 0)
    pdu.set_load("thermal_mgmt", 50)

    print("--- VTOL Hover ---")
    pdu.update(dt=60)  # 1 minute
    print(pdu.status())

    # Simulate cruise
    pdu.set_load("lift_fan_left", 15)
    pdu.set_load("lift_fan_right", 15)
    pdu.set_load("pusher_fan", 70)
    pdu.set_load("thermal_mgmt", 30)

    print("\n--- Cruise ---")
    pdu.update(dt=60)
    print(pdu.status())
