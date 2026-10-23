"""Axial admittance and bounded contact-task state machine (SI units)."""
from dataclasses import dataclass, asdict
import math
import numpy as np


@dataclass(frozen=True)
class ContactConfig:
    mode: str = 'admittance'
    target_force: float = 2.0
    stiffness: float = 800.0
    damping: float = 3.0
    surface_offset: float = 0.0
    surface_tilt_deg: float = -3.0
    surface_present: bool = True
    sensor_noise: float = 0.03
    sensor_bias: float = 0.0
    sensor_delay: float = 0.01
    filter_tau: float = 0.015
    seed: int = 42
    virtual_mass: float = 1.5
    virtual_damping: float = 220.0
    force_limit: float = 6.0
    approach_speed: float = 0.006
    max_speed: float = 0.008
    scan_seconds: float = 8.0
    approach_timeout: float = 5.0
    loss_timeout: float = 0.5
    nominal_surface_z: float = 0.055
    start_x: float = 0.125
    end_x: float = 0.175
    timestep: float = 0.002

    def __post_init__(self):
        if self.mode not in ('position', 'admittance'):
            raise ValueError('mode must be position or admittance')
        if type(self.surface_present) is not bool or type(self.seed) is not int or self.seed < 0:
            raise ValueError('surface_present must be bool and seed a nonnegative integer')
        for name, value in asdict(self).items():
            if name in ('mode', 'surface_present', 'seed'):
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('target_force', 'stiffness', 'damping', 'virtual_mass', 'virtual_damping',
                     'force_limit', 'approach_speed', 'max_speed', 'scan_seconds',
                     'approach_timeout', 'loss_timeout', 'filter_tau', 'timestep'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.sensor_noise < 0 or not 0 <= self.sensor_delay <= 0.2:
            raise ValueError('Invalid sensor noise or delay')
        if not 0 < self.target_force < self.force_limit <= 10:
            raise ValueError('Require 0 < target_force < force_limit <= 10 N')
        if not 300 <= self.stiffness <= 2000 or abs(self.surface_tilt_deg) > 8:
            raise ValueError('Supported spring range: 300..2000 N/m; tilt: +/-8 deg')
        if abs(self.surface_offset) > 0.02 or not 0.12 <= self.start_x < self.end_x <= 0.18:
            raise ValueError('Surface offset or scan endpoints outside supported workspace')
        if self.nominal_surface_z != 0.055 or self.timestep > 0.002:
            raise ValueError('Use the 55 mm fixture and timestep <= 2 ms')
        if self.scan_seconds < 2 or self.approach_speed > 0.01 or self.max_speed > 0.01:
            raise ValueError('Scan must last >= 2 s and vertical speeds <= 10 mm/s')


PROBE_REACH = 0.028  # sphere centre 24 mm from TCP, radius 4 mm


@dataclass(frozen=True)
class ControllerSettings:
    mode: str
    target_force: float
    stiffness: float
    nominal_surface_z: float
    start_x: float
    end_x: float
    timestep: float
    force_limit: float
    virtual_mass: float
    virtual_damping: float
    approach_speed: float
    max_speed: float
    scan_seconds: float
    approach_timeout: float
    loss_timeout: float

    @classmethod
    def from_experiment(cls, config):
        return cls(**{name: getattr(config, name) for name in cls.__dataclass_fields__})


class ContactController:
    """Uses force samples and time only; never receives actual surface geometry."""
    def __init__(self, config):
        self.c = config
        self.start_z = config.nominal_surface_z + PROBE_REACH + 0.008
        self.z = self.start_z
        self.x = config.start_x
        self.velocity = 0.0
        self.phase = 'APPROACH'
        self.phase_start = 0.0
        self.reason = None
        self.loss_since = None
        self.had_contact = False
        self.scan_complete = False

    def _transition(self, phase, time):
        self.phase, self.phase_start = phase, time
        self.velocity = 0.0

    def abort(self, reason, time):
        if self.reason is None:
            self.reason = reason
            self._transition('RETRACT', time)

    def update(self, time, measured_force, raw_force):
        c, dt = self.c, self.c.timestep
        if not np.isfinite([time, measured_force, raw_force]).all():
            self.abort('invalid_sensor', time if math.isfinite(time) else self.phase_start)
        elif raw_force >= c.force_limit and self.phase not in ('RETRACT', 'DONE'):
            self.abort('force_limit', time)
        if self.phase == 'DONE':
            return np.array([self.x, 0.0, self.z])
        if self.phase == 'RETRACT':
            self.z = min(self.start_z, self.z + 0.01 * dt)
            if self.z >= self.start_z - 1e-10:
                self._transition('DONE', time)
            return np.array([self.x, 0.0, self.z])

        self.had_contact |= measured_force > 0.3
        elapsed = time - self.phase_start
        nominal_z = c.nominal_surface_z + PROBE_REACH - c.target_force/c.stiffness
        if self.phase == 'APPROACH':
            if c.mode == 'admittance' and measured_force > 0.3:
                self._transition('SETTLE_FORCE', time)
            elif c.mode == 'position' and self.z <= nominal_z + 1e-10:
                self._transition('SETTLE_FORCE', time)
            elif elapsed >= c.approach_timeout:
                self.abort('approach_timeout', time)
            else:
                self.z -= c.approach_speed * dt
                if c.mode == 'position':
                    self.z = max(nominal_z, self.z)
        elif self.phase in ('SETTLE_FORCE', 'SCAN'):
            if c.mode == 'admittance':
                # Implicit damping update for M*zdd + B*zd = Fz - Ftarget.
                self.velocity = (c.virtual_mass*self.velocity + dt*(measured_force-c.target_force))/(c.virtual_mass+dt*c.virtual_damping)
                self.velocity = float(np.clip(self.velocity, -c.max_speed, c.max_speed))
                self.z += self.velocity*dt
            if self.phase == 'SETTLE_FORCE' and elapsed >= 1.5:
                self._transition('SCAN', time)
            elif self.phase == 'SCAN':
                u = min(1.0, elapsed/c.scan_seconds)
                s = u**3*(10-15*u+6*u*u)
                self.x = c.start_x + (c.end_x-c.start_x)*s
                if elapsed >= c.scan_seconds:
                    self.scan_complete = True
                    self._transition('RETRACT', time)
                elif self.had_contact and measured_force < 0.1:
                    if self.loss_since is None:
                        self.loss_since = time
                    elif time-self.loss_since >= c.loss_timeout:
                        self.abort('contact_lost', time)
                else:
                    self.loss_since = None
        if self.z > self.start_z:
            self.z = self.start_z
            self.abort('travel_limit', time)
        if self.z < self.start_z-0.026:
            self.z = self.start_z-0.026
            self.abort('travel_limit', time)
        return np.array([self.x, 0.0, self.z])
