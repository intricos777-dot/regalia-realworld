# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Copyright (c) 2026 Living Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# Licensed under Living Sin Proprietary License (LSPL-1.0)
# See LICENSE file for details.
# simulation/flight_dynamics.py
"""6-DOF flight dynamics model for the Regalia flying car.

Point-mass translational dynamics plus rigid-body rotational dynamics.
Aerodynamic forces and moments are evaluated in the body frame; thrust is
modelled from the three ducted fans and attitude control allocation.

Body frame: +X forward, +Y starboard, +Z down (NED-like).
Angular convention: positive roll = starboard wing down.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

RHO_SEA_LEVEL = 1.225  # kg/m^3


@dataclass
class VehicleParams:
    """Physical parameters of the vehicle."""
    mass_kg: float = 1600.0
    wing_area_m2: float = 8.5
    wing_span_m: float = 6.4
    chord_m: float = 1.1
    cl_alpha: float = 5.0       # lift curve slope (per radian)
    cd0: float = 0.03           # zero-lift drag coefficient
    k_induced: float = 0.05     # induced drag factor
    cm_alpha: float = -0.5      # pitch moment slope
    max_thrust_n: float = 24000.0  # total max thrust (all fans)
    lift_fan_thrust_n: float = 12000.0  # per lift fan
    pusher_thrust_n: float = 1500.0
    inertia_kgm2: Tuple[float, float, float] = (2500.0, 3500.0, 1500.0)  # Ixx, Iyy, Izz

    # --- Control effectiveness (per unit normalised command) ---
    cl_aileron: float = 0.12    # roll moment slope per unit aileron (-1..1)
    cm_elevator: float = 0.60   # pitch moment slope per unit elevator
    cn_rudder: float = 0.18     # yaw moment slope per unit rudder
    cn_beta: float = 0.06       # weathervane stability (per radian sideslip)

    # --- Rotational damping derivatives (per unit non-dimensional rate) ---
    cl_p: float = -0.35         # roll damping
    cm_q: float = -1.20         # pitch damping
    cn_r: float = -0.25         # yaw damping

    # --- Thrust control allocation ---
    lift_fan_arm_m: float = 0.52      # lateral offset of lift fans (roll arm)
    pitch_authority_nm: float = 3500.0  # fan vectoring pitch moment per unit
    yaw_authority_nm: float = 1800.0    # fan vectoring yaw moment per unit


@dataclass
class ControlInput:
    """Control commands fed to the dynamics.

    ``*_cmd`` are normalised attitude commands (-1..1) used for thrust
    vectoring in VTOL; the aerodynamic surfaces are used in cruise.
    """
    lift_fan_left: float = 0.0   # 0-100% throttle
    lift_fan_right: float = 0.0  # 0-100% throttle
    pusher_fan: float = 0.0      # 0-100% throttle
    aileron: float = 0.0         # -1..1 (positive = roll starboard)
    elevator: float = 0.0        # -1..1 (positive = pitch up)
    rudder: float = 0.0          # -1..1 (positive = yaw starboard)
    roll_cmd: float = 0.0        # -1..1 (reserved for control allocation)
    pitch_cmd: float = 0.0       # -1..1 (fan vectoring, VTOL)
    yaw_cmd: float = 0.0         # -1..1 (fan vectoring, VTOL)

    @classmethod
    def from_dict(cls, d: dict) -> "ControlInput":
        """Build from a throttle dict for backwards compatibility."""
        return cls(
            lift_fan_left=d.get("lift_fan_left", 0.0),
            lift_fan_right=d.get("lift_fan_right", 0.0),
            pusher_fan=d.get("pusher_fan", 0.0),
            aileron=d.get("aileron", 0.0),
            elevator=d.get("elevator", 0.0),
            rudder=d.get("rudder", 0.0),
            roll_cmd=d.get("roll_cmd", 0.0),
            pitch_cmd=d.get("pitch_cmd", 0.0),
            yaw_cmd=d.get("yaw_cmd", 0.0),
        )


@dataclass
class State6DOF:
    """6-DOF state vector."""
    # Position (NED frame)
    x: float = 0.0  # north
    y: float = 0.0  # east
    z: float = 0.0  # down (negative = up)
    # Velocity (body frame)
    u: float = 0.0  # forward
    v: float = 0.0  # right
    w: float = 0.0  # down
    # Attitude (Euler angles, rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Angular rates (body frame, rad/s)
    p: float = 0.0  # roll rate
    q: float = 0.0  # pitch rate
    r: float = 0.0  # yaw rate


class FlightDynamics:
    """Simplified 6-DOF flight dynamics with rotational moments."""

    def __init__(self, params: VehicleParams = None):
        self.params = params or VehicleParams()
        self.state = State6DOF()
        self.time = 0.0

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> List[List[float]]:
        """Body-to-NED rotation matrix."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        R = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ]
        return R

    def _aerodynamics(self, state: State6DOF) -> Tuple[float, float, float]:
        """Compute aerodynamic forces in body frame. Returns (Fx, Fy, Fz)."""
        V = math.sqrt(state.u**2 + state.v**2 + state.w**2)
        if V < 0.1:
            return 0.0, 0.0, 0.0

        alpha = math.atan2(state.w, state.u)
        beta = math.asin(max(-1.0, min(1.0, state.v / V)))

        q_bar = 0.5 * RHO_SEA_LEVEL * V**2
        p = self.params

        CL = p.cl_alpha * alpha
        CD = p.cd0 + p.k_induced * CL**2

        L = q_bar * p.wing_area_m2 * CL
        D = q_bar * p.wing_area_m2 * CD

        Fx = -D * math.cos(alpha) + L * math.sin(alpha)
        Fz = -D * math.sin(alpha) - L * math.cos(alpha)
        Fy = -q_bar * p.wing_area_m2 * 0.1 * beta  # side force

        return Fx, Fy, Fz

    def _aerodynamic_moments(self, state: State6DOF, controls: ControlInput) -> Tuple[float, float, float]:
        """Compute aerodynamic moments (L, M, N) in body frame.

        Includes control-surface effectiveness and rotational damping.
        """
        V = math.sqrt(state.u**2 + state.v**2 + state.w**2)
        if V < 0.1:
            return 0.0, 0.0, 0.0

        p = self.params
        alpha = math.atan2(state.w, state.u)
        beta = math.asin(max(-1.0, min(1.0, state.v / V)))

        q_bar = 0.5 * RHO_SEA_LEVEL * V**2
        S = p.wing_area_m2
        b = p.wing_span_m
        c = p.chord_m

        Cl = (p.cl_aileron * controls.aileron
              + p.cl_p * state.p * b / (2.0 * V))
        Cm = (p.cm_alpha * alpha
              + p.cm_elevator * controls.elevator
              + p.cm_q * state.q * c / (2.0 * V))
        Cn = (p.cn_beta * beta
              + p.cn_rudder * controls.rudder
              + p.cn_r * state.r * b / (2.0 * V))

        L = q_bar * S * b * Cl
        M = q_bar * S * c * Cm
        N = q_bar * S * b * Cn
        return L, M, N

    def _thrust_forces(self, controls: ControlInput) -> Tuple[float, float, float]:
        """Compute thrust forces from fan throttles (body frame: X fwd, Z down)."""
        p = self.params
        lift_left = controls.lift_fan_left / 100.0 * p.lift_fan_thrust_n
        lift_right = controls.lift_fan_right / 100.0 * p.lift_fan_thrust_n
        pusher = controls.pusher_fan / 100.0 * p.pusher_thrust_n

        Fx = pusher
        Fz = -(lift_left + lift_right)  # negative = upward
        Fy = 0.0
        return Fx, Fy, Fz

    def _thrust_moments(self, controls: ControlInput) -> Tuple[float, float, float]:
        """Compute thrust-induced moments (L, M, N) from differential thrust
        and fan vectoring control allocation."""
        p = self.params
        # Differential lift-fan thrust about the longitudinal axis.
        dT = (controls.lift_fan_right - controls.lift_fan_left) / 100.0 * p.lift_fan_thrust_n
        roll = -dT * p.lift_fan_arm_m
        # Fan vectoring (VTOL attitude authority).
        pitch = controls.pitch_cmd * p.pitch_authority_nm
        yaw = controls.yaw_cmd * p.yaw_authority_nm
        return roll, pitch, yaw

    def _gravity(self) -> Tuple[float, float, float]:
        """Gravity force in body frame (Newtons)."""
        g = 9.81 * self.params.mass_kg
        roll = self.state.roll
        pitch = self.state.pitch
        Fx = -g * math.sin(pitch)
        Fy = g * math.cos(pitch) * math.sin(roll)
        Fz = g * math.cos(pitch) * math.cos(roll)
        return Fx, Fy, Fz

    def derivatives(self, state: State6DOF, controls: ControlInput) -> List[float]:
        """Compute state derivatives.

        Returns [dx, dy, dz, du, dv, dw, droll, dpitch, dyaw, dp, dq, dr].
        """
        p = self.params

        Fa = self._aerodynamics(state)
        Ft = self._thrust_forces(controls)
        Fg = self._gravity()

        Fx = Fa[0] + Ft[0] + Fg[0]
        Fy = Fa[1] + Ft[1] + Fg[1]
        Fz = Fa[2] + Ft[2] + Fg[2]

        # Linear accelerations (body frame, includes omega x V)
        du = Fx / p.mass_kg + state.v * state.r - state.w * state.q
        dv = Fy / p.mass_kg + state.w * state.p - state.u * state.r
        dw = Fz / p.mass_kg + state.u * state.q - state.v * state.p

        # Rotational accelerations from aero + thrust moments
        L, M, N = self._aerodynamic_moments(state, controls)
        Lt, Mt, Nt = self._thrust_moments(controls)
        L += Lt
        M += Mt
        N += Nt
        Ixx, Iyy, Izz = p.inertia_kgm2
        dp = L / Ixx
        dq = M / Iyy
        dr = N / Izz

        # Euler angle rates
        droll = (state.p + math.sin(state.roll) * math.tan(state.pitch) * state.q
                 + math.cos(state.roll) * math.tan(state.pitch) * state.r)
        dpitch = math.cos(state.roll) * state.q - math.sin(state.roll) * state.r
        dyaw = (math.sin(state.roll) / math.cos(state.pitch) * state.q
                + math.cos(state.roll) / math.cos(state.pitch) * state.r)

        # Position derivatives (NED frame)
        R = self._rotation_matrix(state.roll, state.pitch, state.yaw)
        dx = R[0][0] * state.u + R[0][1] * state.v + R[0][2] * state.w
        dy = R[1][0] * state.u + R[1][1] * state.v + R[1][2] * state.w
        dz = R[2][0] * state.u + R[2][1] * state.v + R[2][2] * state.w

        return [dx, dy, dz, du, dv, dw, droll, dpitch, dyaw, dp, dq, dr]

    def step(self, controls, dt: float = 0.01):
        """Integrate one time step using RK4.

        ``controls`` may be a :class:`ControlInput` or a throttle dict.
        """
        if not isinstance(controls, ControlInput):
            controls = ControlInput.from_dict(controls)

        s = self.state
        k1 = self.derivatives(s, controls)

        s2 = State6DOF(**{k: s.__dict__[k] + 0.5 * dt * k1[i] for i, k in enumerate(s.__dict__)})
        k2 = self.derivatives(s2, controls)

        s3 = State6DOF(**{k: s.__dict__[k] + 0.5 * dt * k2[i] for i, k in enumerate(s.__dict__)})
        k3 = self.derivatives(s3, controls)

        s4 = State6DOF(**{k: s.__dict__[k] + dt * k3[i] for i, k in enumerate(s.__dict__)})
        k4 = self.derivatives(s4, controls)

        for i, key in enumerate(s.__dict__):
            setattr(s, key, getattr(s, key) + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]))

        self.time += dt


if __name__ == "__main__":
    fd = FlightDynamics()
    # Open-loop VTOL hover: throttle balancing weight on two lift fans.
    p = fd.params
    hover = p.mass_kg * 9.81 / (2.0 * p.lift_fan_thrust_n) * 100.0
    print(f"hover throttle: {hover:.2f}%")
    controls = ControlInput(lift_fan_left=hover, lift_fan_right=hover, pusher_fan=0)
    print("t=0:  z=%.1f  w=%.2f" % (fd.state.z, fd.state.w))
    for i in range(1000):
        fd.step(controls, dt=0.01)
        if i % 100 == 0:
            print("t=%.1f: z=%.1f  w=%.2f  V=%.1f  roll=%.2fdeg" % (
                fd.time, fd.state.z, fd.state.w,
                math.sqrt(fd.state.u**2 + fd.state.v**2 + fd.state.w**2),
                math.degrees(fd.state.roll)))
