"""Four-axis kinematics, using the exact upstream URDF joint frames (metres).

The tool x-axis direction is specified by pitch in the yaw-aligned radial plane.
This is a 4-DOF arm: position plus pitch, not arbitrary six-dimensional pose.
"""
from dataclasses import dataclass
import numpy as np

JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')
LIMITS = np.array([[-np.pi, np.pi], [-1.5, 1.5], [-1.5, 1.4], [-1.7, 1.97]])
BASE_X = 0.012
SHOULDER_Z = 0.0595
UPPER_X, UPPER_Z = 0.024, 0.128
FOREARM = 0.124
TOOL = 0.126


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    rotation: np.ndarray
    pitch: float


def check_joints(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Expected four finite joint angles in radians')
    if np.any(q < LIMITS[:, 0] - 1e-9) or np.any(q > LIMITS[:, 1] + 1e-9):
        raise ValueError(f'Joint angle outside model limits: {q}')
    return q


def forward(q) -> Pose:
    q = check_joints(q)
    yaw = rz(q[0])
    p = np.array([BASE_X, 0, SHOULDER_Z], dtype=float)
    p += yaw @ ry(q[1]) @ np.array([UPPER_X, 0, UPPER_Z])
    p += yaw @ ry(q[1] + q[2]) @ np.array([FOREARM, 0, 0])
    pitch = float(np.sum(q[1:]))
    p += yaw @ ry(pitch) @ np.array([TOOL, 0, 0])
    return Pose(p, yaw @ ry(pitch), pitch)


def joint_points(q):
    q = check_joints(q)
    yaw = rz(q[0])
    shoulder = np.array([BASE_X, 0, SHOULDER_Z])
    elbow = shoulder + yaw @ ry(q[1]) @ [UPPER_X, 0, UPPER_Z]
    wrist = elbow + yaw @ ry(q[1] + q[2]) @ [FOREARM, 0, 0]
    return np.array([[BASE_X, 0, 0], shoulder, elbow, wrist, forward(q).position])


def jacobian(q):
    """Position Jacobian from joint axes and frame origins, not finite differences."""
    q = check_joints(q)
    points = joint_points(q)
    tip = points[-1]
    axis = rz(q[0]) @ [0, 1, 0]
    origins = [points[0], points[1], points[2], points[3]]
    axes = [[0, 0, 1], axis, axis, axis]
    return np.column_stack([np.cross(a, tip - p) for a, p in zip(axes, origins)])


def inverse(position, pitch, seed=None):
    """Enumerate two planar elbow branches and select a valid nearby solution.

    Unreachable targets are rejected; they are never silently projected into reach.
    """
    p = np.asarray(position, dtype=float)
    if p.shape != (3,) or not np.all(np.isfinite(p)) or not np.isfinite(pitch):
        raise ValueError('Expected finite xyz target and pitch')
    dx, y = p[0] - BASE_X, p[1]
    radius = np.hypot(dx, y)
    yaw = float(np.arctan2(y, dx)) if radius > 1e-10 else (0.0 if seed is None else float(seed[0]))
    wrist_r = radius - TOOL * np.cos(pitch)
    wrist_z = p[2] - SHOULDER_Z + TOOL * np.sin(pitch)
    l1 = np.hypot(UPPER_X, UPPER_Z)
    alpha = np.arctan2(UPPER_Z, UPPER_X)
    cos_elbow = (wrist_r**2 + wrist_z**2 - l1**2 - FOREARM**2) / (2 * l1 * FOREARM)
    if abs(cos_elbow) > 1 + 1e-10:
        raise ValueError('Target is outside geometric reach')
    candidates = []
    for theta in [np.arccos(np.clip(cos_elbow, -1, 1)), -np.arccos(np.clip(cos_elbow, -1, 1))]:
        q2 = alpha - np.arctan2(wrist_z, wrist_r) + np.arctan2(FOREARM * np.sin(theta), l1 + FOREARM * np.cos(theta))
        q3 = -theta - alpha
        q = np.array([yaw, q2, q3, pitch - q2 - q3])
        try:
            check_joints(q)
        except ValueError:
            continue
        if np.linalg.norm(forward(q).position - p) < 1e-7:
            candidates.append(q)
    if not candidates:
        raise ValueError('Target violates joint limits for the requested pitch')
    ref = np.zeros(4) if seed is None else check_joints(seed)
    return min(candidates, key=lambda q: np.linalg.norm(q - ref))
