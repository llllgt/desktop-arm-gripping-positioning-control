"""Quintic Hermite action interpolation, independent of ROS Python imports."""
import numpy as np
from scipy.interpolate import BPoly, PPoly
from .continuous import polynomial_bounds, limits4, MotionSample
from .kinematics import LIMITS


class ActionTrajectory:
    def __init__(self, times, positions, velocities, accelerations, *, velocity, acceleration, jerk):
        t = np.asarray(times, dtype=float)
        arrays = [np.asarray(x, dtype=float) for x in (positions, velocities, accelerations)]
        if t.ndim != 1 or len(t) < 2 or not np.all(np.isfinite(t)) or t[0] != 0 or np.any(np.diff(t)<=0):
            raise ValueError('Action sample times must start at zero and increase strictly')
        if any(a.shape != (len(t),4) or not np.all(np.isfinite(a)) for a in arrays):
            raise ValueError('Curved actions require finite position, velocity and acceleration at every point')
        self.duration = float(t[-1])
        self.poly = PPoly.from_bernstein_basis(BPoly.from_derivatives(t,[[a[i] for a in arrays] for i in range(len(t))]))
        polys = [PPoly(self.poly.c[:,:,j],self.poly.x) for j in range(4)]
        low, high = polynomial_bounds(polys)
        if np.any(low[:,0]<LIMITS[:,0]-1e-8) or np.any(high[:,0]>LIMITS[:,1]+1e-8):
            raise ValueError('Action interpolation overshoots a joint limit')
        self.peaks = np.maximum(abs(low[:,1:]),abs(high[:,1:]))
        self.limits = np.column_stack([limits4(velocity,'velocity'),limits4(acceleration,'acceleration'),limits4(jerk,'jerk')])
        if np.any(self.peaks > self.limits*(1+1e-5)+1e-7):
            raise ValueError('Action interpolation exceeds joint velocity, acceleration or jerk limit')

    def sample(self, time):
        if not np.isfinite(time):
            raise ValueError('Action time must be finite')
        t = float(np.clip(time,0,self.duration))
        values = [self.poly(t,nu=n) for n in range(4)]
        if time < 0 or time > self.duration:
            values[1:] = [np.zeros(4) for _ in range(3)]
        return MotionSample(*values)
