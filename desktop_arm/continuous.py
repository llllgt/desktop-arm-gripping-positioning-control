"""Continuous position/pitch splines and polynomial-extrema limit retiming."""
from dataclasses import dataclass
import numpy as np
from numpy.polynomial import Polynomial
from scipy.interpolate import make_interp_spline, PPoly
from .kinematics import inverse, forward, LIMITS


@dataclass(frozen=True)
class MotionSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


def limits4(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape not in ((), (4,)) or not np.all(np.isfinite(a)) or np.any(a <= 0):
        raise ValueError(f'{name} must be a positive finite scalar or four-vector')
    return np.broadcast_to(a, (4,)).copy()


def component_polys(spline):
    return [PPoly.from_spline((spline.t, spline.c[:, i], spline.k)) for i in range(4)]


def polynomial_bounds(polys, orders=(0, 1, 2, 3)):
    """Find extrema at endpoints and real stationary roots, piece by piece.

    Rescale every polynomial interval to [0,1] before root finding. Bounds
    include both sides of knots. This is floating-point root solving, not
    an interval-arithmetic certificate.
    """
    low, high = np.full((4, len(orders)), np.inf), np.full((4, len(orders)), -np.inf)
    for j, pp in enumerate(polys):
        degree = pp.c.shape[0]-1
        for n, width in enumerate(np.diff(pp.x)):
            if width <= 0:
                continue
            coeff = pp.c[:, n][::-1]*width**np.arange(degree+1)
            base = Polynomial(coeff)
            for col, order in enumerate(orders):
                p = base.deriv(order)/width**order
                roots = p.deriv().roots()
                roots = roots[np.abs(roots.imag) < 1e-8].real
                points = np.r_[0., 1., roots[(roots > 0) & (roots < 1)]]
                values = p(points)
                low[j,col] = min(low[j,col], float(values.min()))
                high[j,col] = max(high[j,col], float(values.max()))
    return low, high


def rest_spline(u, values):
    bc = [(order, np.zeros(4)) for order in (1, 2, 3)]
    return make_interp_spline(u, values, k=7, bc_type=(bc, bc))


class ContinuousTransfer:
    def __init__(self, task):
        cfg = task['curve_transfer']
        if not isinstance(cfg, dict) or set(cfg)-{'poses', 'mode', 'jerk_limit', 'ik_samples', 'velocity_feedforward', 'parameterization'}:
            raise ValueError('Unknown curve_transfer parameters')
        if 'poses' not in cfg:
            raise ValueError('curve_transfer requires poses')
        self.mode = cfg.get('mode', 'continuous')
        if self.mode not in ('continuous', 'stop'):
            raise ValueError('Curve mode must be continuous or stop')
        if type(cfg.get('velocity_feedforward', True)) is not bool:
            raise ValueError('velocity_feedforward must be boolean')
        poses = np.asarray(cfg['poses'], dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 4 or not 8 <= len(poses) <= 100 or not np.all(np.isfinite(poses)):
            raise ValueError('Provide 8..100 finite [x,y,z,pitch] poses')
        anchors = [np.r_[np.array(task[key])+[0,0,task['clearance']],task['pitch']] for key in ('pick','place')]
        if not np.allclose(poses[[0,-1]], anchors, atol=1e-9, rtol=0):
            raise ValueError('Curve endpoints must equal lifted pick/place poses and task pitch')
        if task.get('transfer_waypoints'):
            raise ValueError('Use curve_transfer or transfer_waypoints, not both')
        count = cfg.get('ik_samples', 401)
        if type(count) is not int or not 101 <= count <= 2001:
            raise ValueError('ik_samples must be an integer between 101 and 2001')
        self.velocity_limit = limits4(task['max_velocity'], 'velocity')
        self.acceleration_limit = limits4(task['max_acceleration'], 'acceleration')
        self.jerk_limit = limits4(cfg.get('jerk_limit', 8.), 'jerk')
        self.parameterization = cfg.get('parameterization', 'chord')
        if self.parameterization not in ('uniform','chord'):
            raise ValueError('parameterization must be uniform or chord')
        lengths = np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1)
        if np.any(lengths < 1e-6):
            raise ValueError('Consecutive curve points must differ in position by at least 1 micrometre')
        self.nodes = (np.linspace(0.,1.,len(poses)) if self.parameterization=='uniform'
                      else np.r_[0.,np.cumsum(lengths)]/lengths.sum())
        self.reference = rest_spline(self.nodes, poses)
        candidates = np.sort(np.r_[np.linspace(0.,1.,count), self.nodes])
        self.u = candidates[np.r_[True, np.diff(candidates)>1e-12]]
        q, seed = [], None
        for p in self.reference(self.u):
            solution = inverse(p[:3], p[3], seed)
            if seed is not None and np.max(np.abs(solution-seed)) > .2:
                raise ValueError('IK branch discontinuity or undersampled curve')
            q.append(solution); seed = solution
        self.spline = rest_spline(self.u, np.array(q))
        self.start, self.end = self.spline(0.), self.spline(1.)
        # Exact endpoint positions are shared with neighboring task phases.
        self.start = inverse(anchors[0][:3], task['pitch'])
        self.end = inverse(anchors[1][:3], task['pitch'], self.end)
        self.polys = component_polys(self.spline)
        low, high = polynomial_bounds(self.polys)
        if np.any(low[:,0] < LIMITS[:,0]-1e-9) or np.any(high[:,0] > LIMITS[:,1]+1e-9):
            raise ValueError('Interpolated joint curve overshoots a joint limit')
        peaks = np.maximum(np.abs(low[:,1:]), np.abs(high[:,1:]))
        scales = np.column_stack([peaks[:,0]/self.velocity_limit,
                                  np.sqrt(peaks[:,1]/self.acceleration_limit),
                                  np.cbrt(peaks[:,2]/self.jerk_limit)])
        self.duration = float(max(1., scales.max())*1.002)
        self.marker_q = np.array([inverse(p[:3],p[3],self.spline(u)) for u,p in zip(self.nodes,poses)])
        self.stop_segments = []
        if self.mode == 'stop':
            # Same joint v/a/jerk limits; stop at each Cartesian via pose.
            for a,b in zip(self.marker_q,self.marker_q[1:]):
                poly = Polynomial([0,0,0,0,35,-84,70,-20])
                maxima = []
                for n in (1,2,3):
                    d = poly.deriv(n); roots = d.deriv().roots()
                    roots = roots[np.abs(roots.imag)<1e-8].real
                    maxima.append(max(abs(d(np.r_[0,1,roots[(roots>0)&(roots<1)]]))))
                delta = b-a
                duration = max(.4, np.max(maxima[0]*abs(delta)/self.velocity_limit),
                               np.max(np.sqrt(maxima[1]*abs(delta)/self.acceleration_limit)),
                               np.max(np.cbrt(maxima[2]*abs(delta)/self.jerk_limit)))*1.002
                self.stop_segments.append((a,delta,float(duration)))
            self.stops = np.r_[0., np.cumsum([v[2] for v in self.stop_segments])]
            self.duration = float(self.stops[-1])
        # Independently sample Cartesian reconstruction between IK fit nodes.
        dense = (self.u[:-1]+self.u[1:])/2
        actual = np.array([forward(x).position for x in self.spline(dense)])
        self.fit_error_mm = float(np.max(np.linalg.norm(actual-self.reference(dense)[:,:3],axis=1))*1000)
        if self.fit_error_mm > .05:
            raise ValueError('Joint-spline Cartesian reconstruction exceeds 0.05 mm')
        if self.mode == 'continuous':
            self.peaks = peaks/np.array([self.duration,self.duration**2,self.duration**3])
        else:
            self.peaks = np.zeros((4,3))
            for _,delta,duration in self.stop_segments:
                self.peaks = np.maximum(self.peaks, abs(delta[:,None])*np.array(maxima)/duration**np.arange(1,4))

    def sample(self, t):
        if not np.isfinite(t):
            raise ValueError('Sample time must be finite')
        time = float(np.clip(t,0,self.duration))
        if self.mode == 'continuous':
            values = [self.spline(time/self.duration, nu=n)/self.duration**n for n in range(4)]
        else:
            idx = min(len(self.stop_segments)-1, np.searchsorted(self.stops,time,side='right')-1)
            a,delta,duration = self.stop_segments[idx]
            u = (time-self.stops[idx])/duration
            p = Polynomial([0,0,0,0,35,-84,70,-20])
            values = [a+delta*p(u)]+[delta*p.deriv(n)(u)/duration**n for n in (1,2,3)]
        if t <= 0:
            values = [self.start.copy(), *[np.zeros(4) for _ in range(3)]]
        elif t >= self.duration:
            values = [self.end.copy(), *[np.zeros(4) for _ in range(3)]]
        return MotionSample(*values)

    def report(self):
        return dict(mode=self.mode, parameterization=self.parameterization, transfer_duration_s=self.duration, via_poses=len(self.nodes),
                    interior_stops=0 if self.mode=='continuous' else len(self.nodes)-2,
                    joint_spline_degree=7, cartesian_fit_midpoint_error_mm=self.fit_error_mm,
                    peak_velocity_rad_s=self.peaks[:,0].tolist(),
                    peak_acceleration_rad_s2=self.peaks[:,1].tolist(),
                    peak_jerk_rad_s3=self.peaks[:,2].tolist(),
                    retiming='Global duration scaling from polynomial extrema; not time-optimal',
                    joint_limits_checked_between_knots=True)


def build_curve(task):
    import json
    return _cached_curve(json.dumps(task, sort_keys=True))


from functools import lru_cache


@lru_cache(maxsize=32)
def _cached_curve(serialized):
    import json
    return ContinuousTransfer(json.loads(serialized))
