# Watermark: Living_Sin (intricos777-dot) + AI agents (Hermes, Lia, opencode) - collaborative work
# electronics/telemetry.py
"""Telemetry and sensor fusion system.

Collects data from all sensors, fuses IMU data with a complementary filter,
and streams telemetry to ground station.
"""

import time
import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from collections import deque


@dataclass
class IMUData:
    """Raw IMU reading."""
    accel_x: float = 0.0  # m/s^2
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0   # rad/s
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    timestamp: float = 0.0


@dataclass
class GPSData:
    """GPS fix."""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0  # meters (MSL)
    hdop: float = 99.9
    num_sats: int = 0
    timestamp: float = 0.0


@dataclass
class SensorData:
    """Fused sensor data."""
    roll: float = 0.0     # deg
    pitch: float = 0.0    # deg
    yaw: float = 0.0      # deg
    altitude: float = 0.0  # m (MSL)
    airspeed: float = 0.0  # m/s
    vertical_speed: float = 0.0  # m/s
    ground_speed: float = 0.0   # m/s
    battery_soc: float = 100.0
    motor_rpm: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    fault: Optional[str] = None


class ComplementaryFilter:
    """Fuse accelerometer and gyro for attitude estimation."""

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha  # gyro weight
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, imu: IMUData, dt: float) -> tuple:
        """Update attitude estimate. Returns (roll, pitch, yaw) in degrees."""
        # Gyro integration
        self.roll += math.degrees(imu.gyro_x) * dt
        self.pitch += math.degrees(imu.gyro_y) * dt
        self.yaw += math.degrees(imu.gyro_z) * dt

        # Accelerometer tilt (complementary)
        accel_roll = math.degrees(math.atan2(imu.accel_y, imu.accel_z))
        accel_pitch = math.degrees(math.atan2(-imu.accel_x, math.sqrt(imu.accel_y**2 + imu.accel_z**2)))

        # Complementary fusion
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch, self.yaw


class Telemetry:
    """Main telemetry system."""

    def __init__(self):
        self.filter = ComplementaryFilter(alpha=0.98)
        self.imu = IMUData()
        self.gps = GPSData()
        self.sensor = SensorData()
        self._callbacks: List[Callable] = []
        self._log: deque = deque(maxlen=1000)
        self._running = False
        self._rate_hz = 100

    def register_callback(self, cb: Callable):
        self._callbacks.append(cb)

    def _notify(self, data: SensorData):
        for cb in self._callbacks:
            try:
                cb(data)
            except Exception:
                pass

    def update_imu(self, imu: IMUData):
        self.imu = imu
        dt = 0.01 if imu.timestamp == 0 else imu.timestamp - getattr(self, '_prev_imu_t', 0)
        self._prev_imu_t = imu.timestamp
        roll, pitch, yaw = self.filter.update(imu, dt)
        self.sensor.roll = roll
        self.sensor.pitch = pitch
        self.sensor.yaw = yaw

    def update_gps(self, gps: GPSData):
        self.gps = gps
        self.sensor.altitude = gps.alt
        # Estimate vertical speed from GPS altitude
        if hasattr(self, '_prev_gps_alt'):
            dt = gps.timestamp - getattr(self, '_prev_gps_t', gps.timestamp)
            if dt > 0:
                self.sensor.vertical_speed = (gps.alt - self._prev_gps_alt) / dt
        self._prev_gps_alt = gps.alt
        self._prev_gps_t = gps.timestamp

    def update_battery(self, soc: float):
        self.sensor.battery_soc = soc

    def update_motor_rpm(self, rpms: List[float]):
        self.sensor.motor_rpm = rpms

    def pack_frame(self) -> bytes:
        """Pack telemetry into a binary frame for radio transmission."""
        s = self.sensor
        # Simple binary format: header + floats
        # Format: <B ffffff ffff  (1 byte ID, 6 floats, 4 floats)
        frame = struct.pack(
            "<B6f5f",
            0xAA,  # header byte
            s.roll, s.pitch, s.yaw,
            s.altitude, s.airspeed, s.vertical_speed,
            s.ground_speed, s.battery_soc,
            s.motor_rpm[0] if len(s.motor_rpm) > 0 else 0,
            s.motor_rpm[1] if len(s.motor_rpm) > 1 else 0,
            s.motor_rpm[2] if len(s.motor_rpm) > 2 else 0,
        )
        return frame

    def status(self) -> str:
        s = self.sensor
        return (
            f"Telemetry: R={s.roll:6.1f} P={s.pitch:6.1f} Y={s.yaw:6.1f} | "
            f"ALT={s.altitude:7.1f}m  SPD={s.airspeed:5.1f}m/s  "
            f"VS={s.vertical_speed:5.1f}m/s  BAT={s.battery_soc:5.1f}%"
        )


if __name__ == "__main__":
    tel = Telemetry()

    # Simulate some IMU data
    for i in range(100):
        imu = IMUData(
            accel_x=0.1 * math.sin(i * 0.1),
            accel_y=0.0,
            accel_z=9.81,
            gyro_x=0.01,
            gyro_y=0.005,
            gyro_z=0.0,
            timestamp=i * 0.01,
        )
        tel.update_imu(imu)
        if i % 10 == 0:
            tel.update_gps(GPSData(lat=35.0 + i * 0.0001, lon=139.0, alt=100 + i * 0.1))

    tel.update_battery(87.5)
    tel.update_motor_rpm([4200, 4180, 0])

    print(tel.status())
    frame = tel.pack_frame()
    print(f"Frame size: {len(frame)} bytes")
    print(f"Frame hex:  {frame.hex()}")
