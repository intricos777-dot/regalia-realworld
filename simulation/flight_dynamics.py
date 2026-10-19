# simulation/flight_dynamics.py
"""6-DOF flight dynamics model for the Regalia flying car.

Simplified point-mass model with rotational dynamics for simulation
and control system validation.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple


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
    """Simplified 6-DOF flight dynamics."""

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
        p = self.params
        V = math.sqrt(state.u**2 + state.v**2 + state.w**2)
        if V < 0.1:
            return 0.0, 0.0, 0.0

        # Angle of attack
        alpha = math.atan2(state.w, state.u)
        # Sideslip
        beta = math.asin(max(-1, min(1, state.v / V)))

        # Dynamic pressure
        rho = 1.225  # kg/m^3 at sea level
        q_bar = 0.5 * rho * V**2

        # Lift and drag coefficients
        CL = p.cl_alpha * alpha
        CD = p.cd0 + p.k_induced * CL**2

        # Forces in wind frame, then rotate to body
        L = q_bar * p.wing_area_m2 * CL
        D = q_bar * p.wing_area_m2 * CD

        # Body frame forces
        Fx = -D * math.cos(alpha) + L * math.sin(alpha)
        Fz = -D * math.sin(alpha) - L * math.cos(alpha)
        Fy = -q_bar * p.wing_area_m2 * 0.1 * beta  # side force

        return Fx, Fy, Fz

    def _thrust_forces(self, throttle: dict) -> Tuple[float, float, float]:
        """Compute thrust forces from motor throttles."""
        p = self.params
        # Lift fans: vertical thrust (negative Z = up)
        lift_left = throttle.get("lift_fan_left", 0) / 100.0 * p.lift_fan_thrust_n
        lift_right = throttle.get("lift_fan_right", 0) / 100.0 * p.lift_fan_thrust_n
        pusher = throttle.get("pusher_fan", 0) / 100.0 * p.pusher_thrust_n

        # Body frame: X forward, Z down
        Fx = pusher
        Fz = -(lift_left + lift_right)  # negative = upward
        Fy = 0.0
        return Fx, Fy, Fz

    def _gravity(self) -> Tuple[float, float, float]:
        """Gravity force in body frame (Newtons)."""
        g = 9.81 * self.params.mass_kg
        roll = self.state.roll
        pitch = self.state.pitch
        Fx = -g * math.sin(pitch)
        Fy = g * math.cos(pitch) * math.sin(roll)
        Fz = g * math.cos(pitch) * math.cos(roll)
        return Fx, Fy, Fz

    def derivatives(self, state: State6DOF, throttle: dict) -> List[float]:
        """Compute state derivatives. Returns [dx,dy,dz,du,dv,dw,droll,dpitch,dyaw,dp,dq,dr]."""
        p = self.params

        # Forces
        Fa = self._aerodynamics(state)
        Ft = self._thrust_forces(throttle)
        Fg = self._gravity()

        Fx = Fa[0] + Ft[0] + Fg[0]
        Fy = Fa[1] + Ft[1] + Fg[1]
        Fz = Fa[2] + Ft[2] + Fg[2]

        # Linear accelerations
        du = Fx / p.mass_kg + state.v * state.r - state.w * state.q
        dv = Fy / p.mass_kg + state.w * state.p - state.u * state.r
        dw = Fz / p.mass_kg + state.u * state.q - state.v * state.p

        # Angular accelerations (simplified: no moments from aero for now)
        Ixx, Iyy, Izz = p.inertia_kgm2
        dp = 0.0  # would need aero moments
        dq = 0.0
        dr = 0.0

        # Euler angle rates
        droll = state.p + math.sin(state.roll) * math.tan(state.pitch) * state.q + math.cos(state.roll) * math.tan(state.pitch) * state.r
        dpitch = math.cos(state.roll) * state.q - math.sin(state.roll) * state.r
        dyaw = math.sin(state.roll) / math.cos(state.pitch) * state.q + math.cos(state.roll) / math.cos(state.pitch) * state.r

        # Position derivatives (NED frame)
        R = self._rotation_matrix(state.roll, state.pitch, state.yaw)
        dx = R[0][0] * state.u + R[0][1] * state.v + R[0][2] * state.w
        dy = R[1][0] * state.u + R[1][1] * state.v + R[1][2] * state.w
        dz = R[2][0] * state.u + R[2][1] * state.v + R[2][2] * state.w

        return [dx, dy, dz, du, dv, dw, droll, dpitch, dyaw, dp, dq, dr]

    def step(self, throttle: dict, dt: float = 0.01):
        """Integrate one time step using RK4."""
        s = self.state
        k1 = self.derivatives(s, throttle)

        s2 = State6DOF(**{k: s.__dict__[k] + 0.5 * dt * k1[i] for i, k in enumerate(s.__dict__)})
        k2 = self.derivatives(s2, throttle)

        s3 = State6DOF(**{k: s.__dict__[k] + 0.5 * dt * k2[i] for i, k in enumerate(s.__dict__)})
        k3 = self.derivatives(s3, throttle)

        s4 = State6DOF(**{k: s.__dict__[k] + dt * k3[i] for i, k in enumerate(s.__dict__)})
        k4 = self.derivatives(s4, throttle)

        for i, key in enumerate(s.__dict__):
            setattr(s, key, getattr(s, key) + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]))

        self.time += dt


if __name__ == "__main__":
    fd = FlightDynamics()
    # Simulate VTOL hover
    throttle = {"lift_fan_left": 55, "lift_fan_right": 55, "pusher_fan": 0}
    print("t=0:  z=%.1f  w=%.2f" % (fd.state.z, fd.state.w))
    for i in range(1000):
        fd.step(throttle, dt=0.01)
        if i % 100 == 0:
            print("t=%.1f: z=%.1f  w=%.2f  V=%.1f" % (
                fd.time, fd.state.z, fd.state.w,
                math.sqrt(fd.state.u**2 + fd.state.v**2 + fd.state.w**2)))
