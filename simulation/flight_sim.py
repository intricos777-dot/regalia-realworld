# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# simulation/flight_sim.py
"""Integrated closed-loop flight simulator for the Regalia flying car.

Wires the flight controller (electronics.flight_controller) to the 6-DOF
dynamics (simulation.flight_dynamics) in a feedback loop:

    VehicleState -> FlightController -> ControlOutput -> FlightDynamics

Phases: takeoff -> climb -> cruise -> descent -> landing.
"""

import math
import os
import sys
from typing import List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from simulation.flight_dynamics import FlightDynamics, VehicleParams
from simulation.power_system import BatteryPack
from electronics.flight_controller import FlightController, VehicleState
from electronics.power_distribution import PowerDistribution


class FlightSim:
    """Closed-loop flight simulator driven by the flight controller."""

    def __init__(self):
        self.dynamics = FlightDynamics(VehicleParams())
        self.fc = FlightController()
        self.pdu = PowerDistribution()
        self.battery = BatteryPack()

        self.time = 0.0
        self.dt = 0.01
        self.history: List[dict] = []
        self.touchdown = False

    def reset(self):
        self.dynamics = FlightDynamics(VehicleParams())
        self.fc = FlightController()
        self.pdu = PowerDistribution()
        self.time = 0.0
        self.history = []
        self.touchdown = False

    def _vehicle_state(self) -> VehicleState:
        """Map dynamics state (SI, radians) to controller VehicleState (deg)."""
        s = self.dynamics.state
        return VehicleState(
            roll=math.degrees(s.roll),
            pitch=math.degrees(s.pitch),
            yaw=math.degrees(s.yaw),
            roll_rate=math.degrees(s.p),
            pitch_rate=math.degrees(s.q),
            yaw_rate=math.degrees(s.r),
            altitude=-s.z,
            airspeed=math.sqrt(s.u ** 2 + s.v ** 2),
            vertical_speed=-s.w,
        )

    def _step(self):
        """One closed-loop step: sense -> control -> actuate -> power."""
        self.fc.update_state(self._vehicle_state())
        ctrl = self.fc.compute(self.dt)
        self.dynamics.step(ctrl.to_control_input(), self.dt)

        self.pdu.set_load("lift_fan_left", ctrl.lift_fan_left)
        self.pdu.set_load("lift_fan_right", ctrl.lift_fan_right)
        self.pdu.set_load("pusher_fan", ctrl.pusher_fan)
        self.pdu.set_load("avionics", 100)
        self.pdu.set_load("landing_gear", 100 if ctrl.gear_deploy else 0)
        self.pdu.set_load("thermal_mgmt", 50)
        self.pdu.update(self.dt)

        self.time += self.dt
        if -self.dynamics.state.z <= 0.0:
            self.touchdown = True

    def _log(self, label: str = ""):
        s = self.dynamics.state
        alt = -s.z
        spd = math.sqrt(s.u ** 2 + s.v ** 2)
        vs = -s.w
        soc = self.pdu.battery.soc_percent
        pwr = self.pdu.total_power_kw
        self.history.append({
            "t": self.time, "alt": alt, "spd": spd, "vs": vs, "soc": soc,
            "power": pwr, "roll": math.degrees(s.roll),
            "pitch": math.degrees(s.pitch), "label": label,
        })
        print(f"  t={self.time:6.1f}s  alt={alt:7.1f}m  spd={spd:5.1f}m/s  "
              f"vs={vs:5.2f}m/s  SOC={soc:5.1f}%  P={pwr:5.1f}kW {label}")

    def _run(self, label: str, duration_s: float, log_every_s: float = 5.0):
        """Run the loop for a fixed duration, logging periodically."""
        steps = int(duration_s / self.dt)
        last_log = -1e9
        for _ in range(steps):
            self._step()
            if self.time - last_log >= log_every_s:
                self._log(label)
                last_log = self.time
            if self.touchdown and label != "takeoff":
                print("  Touchdown!")
                break

    def run_takeoff(self, duration_s: float = 20.0, target_alt_m: float = 50.0):
        print(f"--- takeoff to {target_alt_m}m ---")
        self.fc.set_mode("vtol")
        self.fc.armed = True
        self.fc.target_altitude = target_alt_m
        self._run("takeoff", duration_s, log_every_s=2.0)

    def run_climb(self, target_alt_m: float = 100.0, max_duration_s: float = 60.0):
        print(f"--- climb to {target_alt_m}m ---")
        self.fc.set_mode("vtol")
        self.fc.target_altitude = target_alt_m
        steps = int(max_duration_s / self.dt)
        last_log = -1e9
        for _ in range(steps):
            self._step()
            if self.time - last_log >= 5.0:
                self._log("climb")
                last_log = self.time
            if -self.dynamics.state.z >= target_alt_m:
                print(f"  Reached target altitude: {-self.dynamics.state.z:.1f}m")
                break

    def run_cruise(self, duration_s: float = 120.0,
                   target_alt_m: float = 120.0, target_airspeed: float = 20.0):
        print(f"--- cruise {duration_s}s @ {target_airspeed} m/s ---")
        self.fc.set_mode("cruise")
        self.fc.target_altitude = target_alt_m
        self.fc.target_airspeed = target_airspeed
        self._run("cruise", duration_s, log_every_s=10.0)

    def run_descent(self, target_alt_m: float = 10.0, max_duration_s: float = 60.0):
        print(f"--- descent to {target_alt_m}m ---")
        self.fc.set_mode("vtol")
        self.fc.target_altitude = target_alt_m
        steps = int(max_duration_s / self.dt)
        last_log = -1e9
        for _ in range(steps):
            self._step()
            if self.time - last_log >= 5.0:
                self._log("descent")
                last_log = self.time
            if -self.dynamics.state.z <= target_alt_m:
                print(f"  Reached target altitude: {-self.dynamics.state.z:.1f}m")
                break

    def run_landing(self, max_duration_s: float = 60.0):
        print("--- landing ---")
        self.fc.set_mode("vtol")
        self.fc.target_altitude = 0.0
        steps = int(max_duration_s / self.dt)
        last_log = -1e9
        for _ in range(steps):
            self._step()
            if self.time - last_log >= 5.0:
                self._log("landing")
                last_log = self.time
            if self.touchdown:
                print("  Touchdown!")
                break

    def run_flight_plan(self):
        """Run a full closed-loop flight plan."""
        print("=" * 60)
        print("FLIGHT SIMULATION — Regalia Real-World Equivalent (closed loop)")
        print("=" * 60)
        print(f"Hover throttle: {self.fc.hover_throttle:.1f}%")

        self.run_takeoff(duration_s=20.0, target_alt_m=50.0)
        self.run_climb(target_alt_m=100.0, max_duration_s=60.0)
        self.run_cruise(duration_s=120.0, target_alt_m=120.0, target_airspeed=20.0)
        self.run_descent(target_alt_m=10.0, max_duration_s=90.0)
        self.run_landing(max_duration_s=90.0)

        s = self.dynamics.state
        print("\n" + "=" * 60)
        print("FLIGHT SUMMARY")
        print("=" * 60)
        print(f"Total time:    {self.time:.1f}s")
        print(f"Final alt:     {-s.z:.1f}m")
        print(f"Final speed:   {math.sqrt(s.u ** 2 + s.v ** 2):.1f} m/s")
        print(f"Final pitch:   {math.degrees(s.pitch):.1f} deg")
        print(f"Touchdown:     {self.touchdown}")
        print(f"Battery SOC:   {self.pdu.battery.soc_percent:.1f}%")
        print(f"Total power:   {self.pdu.total_power_kw:.1f} kW")
        return self.history


if __name__ == "__main__":
    sim = FlightSim()
    sim.run_flight_plan()
