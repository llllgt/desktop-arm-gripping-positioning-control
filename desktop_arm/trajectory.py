"""Synchronized polynomial joint trajectories with analytical limit retiming."""
from dataclasses import dataclass
import numpy as np
from .kinematics import check_joints


@dataclass(frozen=True)
class Sample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


class Segment:
    def __init__(self, start, end, *, order=5, velocity=0.8, acceleration=1.5, min_duration=0.4):
        self.start = check_joints(start).copy()
        self.end = check_joints(end).copy()
        if order not in (3, 5):
            raise ValueError('Polynomial order must be 3 or 5')
        self.order = order
        self.delta = self.end - self.start
        vmax = np.broadcast_to(np.asarray(velocity, dtype=float), (4,))
        amax = np.broadcast_to(np.asarray(acceleration, dtype=float), (4,))
        if not np.all(np.isfinite(vmax)) or not np.all(np.isfinite(amax)) or np.any(vmax <= 0) or np.any(amax <= 0) or not np.isfinite(min_duration) or min_duration <= 0:
            raise ValueError('Trajectory limits and minimum duration must be positive finite values')
        # Exact maxima of ds/du and abs(d2s/du2) over u in [0,1].
        sv, sa = (1.5, 6.0) if order == 3 else (1.875, 10 / np.sqrt(3))
        self.duration = float(max(min_duration, np.max(sv * np.abs(self.delta) / vmax), np.max(np.sqrt(sa * np.abs(self.delta) / amax))))

    def sample(self, t):
        if not np.isfinite(t):
            raise ValueError('Sample time must be finite')
        u = float(np.clip(t / self.duration, 0, 1))
        if self.order == 5:
            s = 10*u**3 - 15*u**4 + 6*u**5
            ds = 30*u**2 - 60*u**3 + 30*u**4
            dds = 60*u - 180*u**2 + 120*u**3
        else:
            s = 3*u**2 - 2*u**3
            ds = 6*u - 6*u**2
            dds = 6 - 12*u
        if t < 0 or t > self.duration:
            ds = dds = 0
        return Sample(self.start + s*self.delta, ds*self.delta/self.duration, dds*self.delta/self.duration**2)
