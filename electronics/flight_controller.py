# electronics/flight_controller.py
"""Flight control computer (FCC) backend for the Regalia flying car.

Implements PID control loops for attitude, altitude, and heading.
Triple-redundant architecture with voting logic.
"""

import time
import math
from dataclasses import dataclass, field
from typing import List, Tuple


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
    gear_deploy: bool = False
    wing_deploy: bool = False


class PIDController:
    """Standard PID controller with anti-windup."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
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
            if self.prev_error is not None and dt > 0:
                d_term = -self.gains.kd * (measurement - self.prev_error) / dt
            self.prev_error = measurement

            output = p_term + i_term + d_term
            output = max(-self.gains.output_limit, min(self.gains.output_limit, output))
            return output
        return 0.0


class FlightController:
    """Main flight control computer."""

    def __init__(self):
        # Attitude controllers
        self.roll_pid = PIDController(PIDGains(kp=2.0, ki=0.1, kd=0.3))
        self.pitch_pid = PIDController(PIDGains(kp=2.0, ki=0.1, kd=0.3))
        self.yaw_pid = PIDController(PIDGains(kp=1.5, ki=0.05, kd=0.2))

        # Rate controllers (inner loop)
        self.roll_rate_pid = PIDController(PIDGains(kp=0.8, ki=0.02, kd=0.1))
        self.pitch_rate_pid = PIDController(PIDGains(kp=0.8, ki=0.02, kd=0.1))
        self.yaw_rate_pid = PIDController(PIDGains(kp=0.6, ki=0.01, kd=0.05))

        # Altitude hold (cascade: outer loop = vertical speed, inner = throttle)
        self.altitude_pid = PIDController(PIDGains(kp=0.08, ki=0.005, kd=0.15))
        self.vertical_speed_pid = PIDController(PIDGains(kp=0.8, ki=0.05, kd=0.0))
        self.hover_throttle = 65.0  # % throttle for level hover (1600kg, 2x12000N)

        # Airspeed control (separate from pitch attitude)
        self.airspeed_pid = PIDController(PIDGains(kp=0.1, ki=0.01, kd=0.05))

        # State
        self.state = VehicleState()
        self.armed = False
        self.mode = "road"  # road | vtol | cruise

        # Setpoints
        self.target_altitude = 0.0
        self.target_heading = 0.0
        self.target_airspeed = 0.0

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

    def compute(self, dt: float = 0.01) -> ControlOutput:
        """Main control computation. Returns control outputs."""
        if not self.armed:
            return ControlOutput()

        out = ControlOutput()
        s = self.state

        if self.mode == "road":
            # Road mode: everything stowed
            out.gear_deploy = True
            out.wing_deploy = False
            return out

        elif self.mode == "vtol":
            # VTOL: cascade altitude control (outer=alt, inner=vertical speed)
            out.gear_deploy = False
            out.wing_deploy = False

            # Outer loop: altitude -> target vertical speed
            target_vs = max(-3.0, min(3.0, self.altitude_pid.update(self.target_altitude, s.altitude, dt)))

            # Inner loop: vertical speed -> throttle adjustment
            vs_cmd = self.vertical_speed_pid.update(target_vs, s.vertical_speed, dt)
            base_throttle = self.hover_throttle + vs_cmd * 5.0
            base_throttle = max(0.0, min(100.0, base_throttle))

            # Roll control -> differential thrust
            roll_cmd = self.roll_pid.update(0.0, s.roll, dt)
            out.lift_fan_left = max(0, min(100, base_throttle - roll_cmd * 20))
            out.lift_fan_right = max(0, min(100, base_throttle + roll_cmd * 20))

            # Pitch control -> differential (forward fans vs rear)
            pitch_cmd = self.pitch_pid.update(0.0, s.pitch, dt)

            # Yaw control -> differential thrust left/right
            yaw_cmd = self.yaw_pid.update(self.target_heading, s.yaw, dt)

            # Pusher fan off in VTOL
            out.pusher_fan = 0.0

        elif self.mode == "cruise":
            # Cruise: wings deployed, pusher fan provides forward thrust
            out.gear_deploy = False
            out.wing_deploy = True

            # Airspeed control -> pusher fan throttle (separate PID)
            spd_cmd = self.airspeed_pid.update(self.target_airspeed, s.airspeed, dt)
            out.pusher_fan = max(0, min(100, 30.0 + spd_cmd * 10))

            # Lift fans supplement wing lift (wing provides ~13% at 30 m/s)
            out.lift_fan_left = 57.0
            out.lift_fan_right = 57.0

            # Roll/pitch/yaw control surfaces
            roll_cmd = self.roll_pid.update(0.0, s.roll, dt)
            pitch_cmd = self.pitch_pid.update(0.0, s.pitch, dt)
            yaw_cmd = self.yaw_pid.update(self.target_heading, s.yaw, dt)

            out.aileron_left = max(-1, min(1, -roll_cmd))
            out.aileron_right = max(-1, min(1, roll_cmd))
            out.elevator = max(-1, min(1, pitch_cmd))
            out.rudder = max(-1, min(1, yaw_cmd))

        return out


class RedundantFCC:
    """Triple-redundant flight control with voting."""

    def __init__(self):
        self.channels = [FlightController() for _ in range(3)]
        self.active_channel = 0

    def compute(self) -> ControlOutput:
        """Compute using all 3 channels, vote on outputs."""
        outputs = [ch.compute() for ch in self.channels]

        # Simple median voting on lift fan throttle
        left_throttles = sorted([o.lift_fan_left for o in outputs])
        right_throttles = sorted([o.lift_fan_right for o in outputs])
        pusher_throttles = sorted([o.pusher_fan for o in outputs])

        # Use median channel
        median_idx = 1
        result = outputs[median_idx]
        result.lift_fan_left = left_throttles[1]
        result.lift_fan_right = right_throttles[1]
        result.pusher_fan = pusher_throttles[1]

        return result

    def update_all_states(self, state: VehicleState):
        for ch in self.channels:
            ch.update_state(state)


# ---- Demo / test ----
if __name__ == "__main__":
    fc = FlightController()
    fc.armed = True

    for mode in ("road", "vtol", "cruise"):
        fc.set_mode(mode)
        fc.target_altitude = 100.0
        fc.target_heading = 90.0
        fc.target_airspeed = 30.0

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
              f"Elevator: {out.elevator:.2f}  Rudder: {out.rudder:.2f}")
        print()
