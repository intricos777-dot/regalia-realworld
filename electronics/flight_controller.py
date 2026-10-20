# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# electronics/flight_controller.py
"""Flight control computer (FCC) backend for the Regalia flying car.

Implements PID control loops for attitude, altitude, and heading.
Triple-redundant architecture with voting logic.

Modes:
- road   : gear down, everything stowed
- vtol   : altitude cascade + differential thrust roll + fan vectoring
- cruise  : airspeed/pitch hold + aerodynamic control surfaces
"""

import math
import time
from dataclasses import dataclass
from typing import List


@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.1
    kd: float = 0.05
    integral_limit: float = 10.0
    output_limit: float = 1.0


@dataclass
class VehicleState:
    """Current vehicle state estimate."""
    roll: float = 0.0      # degrees
    pitch: float = 0.0     # degrees
    yaw: float = 0.0       # degrees (heading)
    roll_rate: float = 0.0   # deg/s
    pitch_rate: float = 0.0  # deg/s
    yaw_rate: float = 0.0    # deg/s
    altitude: float = 0.0  # meters
    airspeed: float = 0.0  # m/s
    vertical_speed: float = 0.0  # m/s
    timestamp: float = 0.0


@dataclass
class ControlOutput:
    """Control surface / motor commands."""
    lift_fan_left: float = 0.0   # 0-100% throttle
    lift_fan_right: float = 0.0  # 0-100% throttle
    pusher_fan: float = 0.0      # 0-100% throttle
    aileron_left: float = 0.0    # -1 to 1
    aileron_right: float = 0.0   # -1 to 1
    elevator: float = 0.0        # -1 to 1
    rudder: float = 0.0          # -1 to 1
    # Normalised attitude commands for thrust-vectoring allocation (-1..1)
    pitch_cmd: float = 0.0
    yaw_cmd: float = 0.0
    gear_deploy: bool = False
    wing_deploy: bool = False

    def to_control_input(self):
        """Map to a dynamics ControlInput (simulation.flight_dynamics)."""
        from simulation.flight_dynamics import ControlInput
        return ControlInput(
            lift_fan_left=self.lift_fan_left,
            lift_fan_right=self.lift_fan_right,
            pusher_fan=self.pusher_fan,
            aileron=(self.aileron_right - self.aileron_left) / 2.0,
            elevator=self.elevator,
            rudder=self.rudder,
            pitch_cmd=self.pitch_cmd,
            yaw_cmd=self.yaw_cmd,
        )


class PIDController:
    """Standard PID controller with anti-windup."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.integral = 0.0
        # None until the first sample: avoids a derivative kick from a
        # bogus initial "previous measurement".
        self.prev_error = None
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None

    def update(self, setpoint: float, measurement: float, dt: float = None) -> float:
        now = time.monotonic()
        if dt is None:
            if self.prev_time is None:
                dt = 0.0
            else:
                dt = now - self.prev_time
        self.prev_time = now

        error = setpoint - measurement

        if dt > 0:
            # Proportional
            p_term = self.gains.kp * error

            # Integral with anti-windup
            self.integral += error * dt
            self.integral = max(-self.gains.integral_limit,
                                min(self.gains.integral_limit, self.integral))
            i_term = self.gains.ki * self.integral

            # Derivative (on measurement, not error, to avoid derivative kick)
            d_term = 0.0
            if self.prev_error is not None:
                d_term = -self.gains.kd * (measurement - self.prev_error) / dt
            self.prev_error = measurement

            output = p_term + i_term + d_term
            output = max(-self.gains.output_limit, min(self.gains.output_limit, output))
            return output
        return 0.0


class FlightController:
    """Main flight control computer."""

    # Vehicle constants (mirror simulation.flight_dynamics.VehicleParams)
    mass_kg: float = 1600.0
    lift_fan_thrust_n: float = 12000.0
    wing_area_m2: float = 8.5
    cl_alpha: float = 5.0
    rho: float = 1.225

    def __init__(self):
        # Attitude controllers (outer loop, degrees)
        self.roll_pid = PIDController(PIDGains(kp=0.08, ki=0.005, kd=0.02))
        self.pitch_pid = PIDController(PIDGains(kp=0.08, ki=0.005, kd=0.02))
        self.yaw_pid = PIDController(PIDGains(kp=0.06, ki=0.004, kd=0.02))

        # Altitude cascade (outer=alt->vs, inner=vs->throttle %)
        self.altitude_pid = PIDController(
            PIDGains(kp=0.2, ki=0.02, kd=0.3, integral_limit=50.0, output_limit=5.0))
        self.vertical_speed_pid = PIDController(
            PIDGains(kp=2.0, ki=0.3, kd=0.1, integral_limit=50.0, output_limit=35.0))

        # Airspeed control (cruise pusher)
        self.airspeed_pid = PIDController(
            PIDGains(kp=0.6, ki=0.05, kd=0.1, integral_limit=50.0, output_limit=70.0))

        # Derived hover throttle: balances weight on two lift fans
        self.hover_throttle = self.mass_kg * 9.81 / (2.0 * self.lift_fan_thrust_n) * 100.0

        # State
        self.state = VehicleState()
        self.armed = False
        self.mode = "road"  # road | vtol | cruise

        # Setpoints
        self.target_altitude = 0.0
        self.target_heading = 0.0
        self.target_airspeed = 0.0
        self.target_pitch = 8.0  # degrees, cruise trim AoA for wing lift

    def set_mode(self, mode: str):
        if mode not in ("road", "vtol", "cruise"):
            raise ValueError(f"Invalid mode: {mode}")
        self.mode = mode
        # Reset integrators on mode change
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.altitude_pid.reset()
        self.vertical_speed_pid.reset()
        self.airspeed_pid.reset()

    def update_state(self, state: VehicleState):
        self.state = state

    def _compute_vtol(self, s: VehicleState, out: ControlOutput, dt: float):
        """VTOL: altitude cascade + differential thrust roll + vectoring."""
        out.gear_deploy = False
        out.wing_deploy = False

        # Outer loop: altitude -> target vertical speed
        target_vs = self.altitude_pid.update(self.target_altitude, s.altitude, dt)
        # Inner loop: vertical speed -> throttle
        vs_cmd = self.vertical_speed_pid.update(target_vs, s.vertical_speed, dt)
        base = max(0.0, min(100.0, self.hover_throttle + vs_cmd))

        # Roll via differential thrust: right fan > left fan -> roll left,
        # so right = base - roll_cmd, left = base + roll_cmd (roll_cmd = -kp*roll).
        roll_cmd = self.roll_pid.update(0.0, s.roll, dt)
        out.lift_fan_left = max(0.0, min(100.0, base + roll_cmd * 20.0))
        out.lift_fan_right = max(0.0, min(100.0, base - roll_cmd * 20.0))

        # Pitch and yaw via fan vectoring (VTOL attitude authority)
        out.pitch_cmd = self.pitch_pid.update(0.0, s.pitch, dt)
        out.yaw_cmd = self.yaw_pid.update(self.target_heading, s.yaw, dt)

        # Pusher off in VTOL; no aerodynamic surfaces (no airflow)
        out.pusher_fan = 0.0

    def _compute_cruise(self, s: VehicleState, out: ControlOutput, dt: float):
        """Cruise: adaptive lift split + airspeed/pitch hold + aero surfaces."""
        out.gear_deploy = False
        out.wing_deploy = True

        # Wing lift share of weight at current speed and pitch
        weight = self.mass_kg * 9.81
        q_bar = 0.5 * self.rho * s.airspeed ** 2
        alpha_rad = math.radians(max(0.0, s.pitch))
        wing_lift = q_bar * self.wing_area_m2 * self.cl_alpha * alpha_rad
        wing_frac = max(0.0, min(0.5, wing_lift / weight))
        fan_throttle = max(0.0, min(100.0, self.hover_throttle * (1.0 - wing_frac)))
        out.lift_fan_left = fan_throttle
        out.lift_fan_right = fan_throttle

        # Airspeed -> pusher fan
        spd_cmd = self.airspeed_pid.update(self.target_airspeed, s.airspeed, dt)
        out.pusher_fan = max(0.0, min(100.0, 25.0 + spd_cmd))

        # Altitude hold biases the pitch target
        alt_err = self.target_altitude - s.altitude
        pitch_bias = max(-5.0, min(5.0, 0.05 * alt_err))
        pitch_cmd = self.pitch_pid.update(self.target_pitch + pitch_bias, s.pitch, dt)
        roll_cmd = self.roll_pid.update(0.0, s.roll, dt)
        yaw_cmd = self.yaw_pid.update(self.target_heading, s.yaw, dt)

        out.aileron_left = max(-1.0, min(1.0, -roll_cmd))
        out.aileron_right = max(-1.0, min(1.0, roll_cmd))
        out.elevator = max(-1.0, min(1.0, pitch_cmd))
        out.rudder = max(-1.0, min(1.0, yaw_cmd))

    def compute(self, dt: float = 0.01) -> ControlOutput:
        """Main control computation. Returns control outputs."""
        if not self.armed:
            return ControlOutput()

        out = ControlOutput()
        s = self.state

        if self.mode == "road":
            out.gear_deploy = True
            out.wing_deploy = False
            return out
        elif self.mode == "vtol":
            self._compute_vtol(s, out, dt)
        elif self.mode == "cruise":
            self._compute_cruise(s, out, dt)

        return out


class RedundantFCC:
    """Triple-redundant flight control with median voting on all outputs."""

    # Fields on ControlOutput that are voted on
    _VOTE_FIELDS = (
        "lift_fan_left", "lift_fan_right", "pusher_fan",
        "aileron_left", "aileron_right", "elevator", "rudder",
        "pitch_cmd", "yaw_cmd",
    )

    def __init__(self):
        self.channels = [FlightController() for _ in range(3)]
        self.active_channel = 0

    def compute(self, dt: float = 0.01) -> ControlOutput:
        """Compute on all 3 channels, take median of each output field."""
        outputs = [ch.compute(dt) for ch in self.channels]
        result = outputs[0]
        for field in self._VOTE_FIELDS:
            values = sorted(getattr(o, field) for o in outputs)
            setattr(result, field, values[1])
        # Discrete outputs come from the majority channel (channel 0).
        result.gear_deploy = outputs[0].gear_deploy
        result.wing_deploy = outputs[0].wing_deploy
        return result

    def set_mode(self, mode: str):
        for ch in self.channels:
            ch.set_mode(mode)

    def update_all_states(self, state: VehicleState):
        for ch in self.channels:
            ch.update_state(state)


# ---- Demo / test ----
if __name__ == "__main__":
    fc = FlightController()
    fc.armed = True
    print(f"Hover throttle: {fc.hover_throttle:.1f}%")

    for mode in ("road", "vtol", "cruise"):
        fc.set_mode(mode)
        fc.target_altitude = 100.0
        fc.target_heading = 90.0
        fc.target_airspeed = 20.0

        state = VehicleState(
            roll=5.0, pitch=-3.0, yaw=88.0,
            roll_rate=0.5, pitch_rate=-0.2, yaw_rate=0.1,
            altitude=95.0, airspeed=28.0, vertical_speed=0.5,
        )
        fc.update_state(state)
        out = fc.compute()
        print(f"Mode: {mode}")
        print(f"  Lift L: {out.lift_fan_left:.1f}%  Lift R: {out.lift_fan_right:.1f}%  "
              f"Pusher: {out.pusher_fan:.1f}%")
        print(f"  Aileron L: {out.aileron_left:.2f}  R: {out.aileron_right:.2f}  "
              f"Elevator: {out.elevator:.2f}  Rudder: {out.rudder:.2f}  "
              f"PitchCmd: {out.pitch_cmd:.2f}  YawCmd: {out.yaw_cmd:.2f}")
        print()
