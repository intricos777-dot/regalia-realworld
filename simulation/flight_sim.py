# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# simulation/flight_sim.py
"""Integrated flight simulator for the Regalia flying car.

Uses pure open-loop throttle profiles (standard for VTOL validation):
- Takeoff: 75% lift throttle for 5s
- Climb: 80% lift throttle until target altitude
- Cruise: 57% lift + 30% pusher throttle
- Descent: 62% lift throttle
- Landing: 58% lift throttle until ground contact
"""

import math
import os
import sys
from typing import List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from simulation.flight_dynamics import FlightDynamics, VehicleParams
from simulation.power_system import BatteryPack, PowerBudget
from electronics.power_distribution import PowerDistribution


class FlightSim:
    """Open-loop flight simulator using fixed throttle profiles."""

    def __init__(self):
        self.dynamics = FlightDynamics(VehicleParams())
        self.pdu = PowerDistribution()
        self.battery = BatteryPack()

        self.time = 0.0
        self.dt = 0.01
        self.history: List[dict] = []

    def reset(self):
        self.dynamics = FlightDynamics(VehicleParams())
        self.time = 0.0
        self.history = []

    def _step(self, throttle: dict):
        """One simulation step."""
        self.dynamics.step(throttle, self.dt)
        self.pdu.set_load("lift_fan_left", throttle.get("lift_fan_left", 0))
        self.pdu.set_load("lift_fan_right", throttle.get("lift_fan_right", 0))
        self.pdu.set_load("pusher_fan", throttle.get("pusher_fan", 0))
        self.pdu.set_load("avionics", 100)
        self.pdu.set_load("landing_gear", 0)
        self.pdu.set_load("thermal_mgmt", 50)
        self.pdu.update(self.dt)
        self.time += self.dt

    def _log(self, label: str = ""):
        s = self.dynamics.state
        spd = math.sqrt(s.u**2 + s.v**2)
        alt = -s.z
        vs = -s.w
        soc = self.pdu.battery.soc_percent
        pwr = self.pdu.total_power_kw
        self.history.append({
            "t": self.time, "alt": alt, "spd": spd, "vs": vs,
            "soc": soc, "power": pwr, "label": label,
        })
        print(f"  t={self.time:5.1f}s  alt={alt:6.1f}m  spd={spd:5.1f}m/s  "
              f"vs={vs:5.2f}m/s  SOC={soc:5.1f}%  P={pwr:5.1f}kW {label}")

    def run_takeoff(self, duration_s: float = 5.0):
        """Takeoff: climb vertically at 68% lift throttle."""
        print(f"--- takeoff ({duration_s}s) ---")
        steps = int(duration_s / self.dt)
        for i in range(steps):
            self._step({"lift_fan_left": 68, "lift_fan_right": 68, "pusher_fan": 0})
            if i % 500 == 0:
                self._log("takeoff")

    def run_climb(self, target_alt_m: float = 100.0, max_duration_s: float = 30.0):
        """Climb to target altitude at 70% lift throttle."""
        print(f"--- climb to {target_alt_m}m ---")
        steps = int(max_duration_s / self.dt)
        for i in range(steps):
            self._step({"lift_fan_left": 70, "lift_fan_right": 70, "pusher_fan": 0})
            if i % 500 == 0:
                self._log("climb")
            s = self.dynamics.state
            if -s.z >= target_alt_m:
                print(f"  Reached target altitude: {-s.z:.1f}m")
                break

    def run_cruise(self, duration_s: float = 60.0):
        """Cruise at 65% lift + 20% pusher throttle."""
        print(f"--- cruise ({duration_s}s) ---")
        steps = int(duration_s / self.dt)
        for i in range(steps):
            self._step({"lift_fan_left": 65, "lift_fan_right": 65, "pusher_fan": 20})
            if i % 500 == 0:
                self._log("cruise")

    def run_descent(self, duration_s: float = 30.0):
        """Descend at 58% lift throttle."""
        print(f"--- descent ({duration_s}s) ---")
        steps = int(duration_s / self.dt)
        for i in range(steps):
            self._step({"lift_fan_left": 58, "lift_fan_right": 58, "pusher_fan": 0})
            if i % 500 == 0:
                self._log("descent")

    def run_landing(self, duration_s: float = 10.0):
        """Land at 60% lift throttle."""
        print(f"--- landing ({duration_s}s) ---")
        steps = int(duration_s / self.dt)
        for i in range(steps):
            self._step({"lift_fan_left": 60, "lift_fan_right": 60, "pusher_fan": 0})
            if i % 200 == 0:
                self._log("landing")
            s = self.dynamics.state
            if -s.z < 0.5:
                print(f"  Touchdown!")
                break

    def run_flight_plan(self, verbose: bool = True):
        """Run a full flight plan."""
        print("=" * 60)
        print("FLIGHT SIMULATION — Regalia Real-World Equivalent")
        print("=" * 60)

        self.run_takeoff(duration_s=5.0)
        self.run_climb(target_alt_m=100.0, max_duration_s=30.0)
        self.run_cruise(duration_s=60.0)
        self.run_descent(duration_s=30.0)
        self.run_landing(duration_s=10.0)

        # Summary
        s = self.dynamics.state
        print("\n" + "=" * 60)
        print("FLIGHT SUMMARY")
        print("=" * 60)
        print(f"Total time:   {self.time:.1f}s")
        print(f"Final alt:    {-s.z:.1f}m")
        print(f"Final speed:  {math.sqrt(s.u**2 + s.v**2):.1f}m/s")
        print(f"Battery SOC:  {self.pdu.battery.soc_percent:.1f}%")
        print(f"Total power:  {self.pdu.total_power_kw:.1f} kW")

        return self.history


if __name__ == "__main__":
    sim = FlightSim()
    history = sim.run_flight_plan(verbose=True)
