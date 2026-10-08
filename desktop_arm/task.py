"""Explicit manipulation state schedule shared by native and ROS execution."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np
from .kinematics import inverse
from .trajectory import Segment
from .paths import project_root


@dataclass
class Phase:
    name: str
    segment: Segment
    gripper_start: float
    gripper_end: float

    def command(self, time):
        # Smooth gripper travel uses the same quintic scalar profile.
        u = float(np.clip(time / self.segment.duration, 0, 1))
        s = 10*u**3 - 15*u**4 + 6*u**5
        return self.segment.sample(time), self.gripper_start + s*(self.gripper_end - self.gripper_start)


def load_task(path=None):
    task = json.loads((project_root() / 'config/task.json' if path is None else Path(path)).read_text(encoding='utf-8'))
    return validate_task(task)


def validate_task(task):
    """Validate units/geometry before either native or ROS task planning."""
    if not isinstance(task, dict):
        raise ValueError('Task configuration must be a JSON object')
    required = ('pick', 'place', 'pitch', 'clearance', 'workpiece_size', 'workpiece_mass',
                'gripper_open', 'gripper_closed', 'max_velocity', 'max_acceleration', 'dwell')
    missing = [key for key in required if key not in task]
    if missing:
        raise ValueError('Missing task parameters: ' + ', '.join(missing))
    for key in ('pick', 'place', 'workpiece_size', 'initial_workpiece_offset'):
        value = np.asarray(task.get(key, [0, 0, 0]), dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f'{key} must contain three finite values in metres')
    for key in ('pitch', 'clearance', 'workpiece_mass', 'dwell', 'gripper_open', 'gripper_closed', 'contact_friction'):
        value = np.asarray(task.get(key, 1.2), dtype=float)
        if value.shape != () or not np.isfinite(value):
            raise ValueError(f'{key} must be a finite scalar')
    if np.any(np.asarray(task['workpiece_size']) <= 0):
        raise ValueError('workpiece_size must be positive')
    if any(task[key] <= 0 for key in ('clearance', 'workpiece_mass', 'dwell')):
        raise ValueError('clearance, workpiece_mass and dwell must be positive')
    if task.get('contact_friction', 1.2) < 0:
        raise ValueError('contact_friction must be nonnegative')
    for key in ('max_velocity', 'max_acceleration'):
        value = np.asarray(task[key], dtype=float)
        if value.shape not in ((), (4,)) or not np.all(np.isfinite(value)) or np.any(value <= 0):
            raise ValueError(f'{key} must be positive, scalar or four joint limits')
    if not (-.01 <= task['gripper_closed'] < task['gripper_open'] <= .019):
        raise ValueError('Gripper targets must obey -0.01 <= closed < open <= 0.019 m')
    design = json.loads((project_root()/'cad/design.json').read_text(encoding='utf-8'))
    for key in ('pick', 'place'):
        if not np.isclose(task[key][2] - task['workpiece_size'][2]/2,
                          design['fixture']['total_height']/1000, rtol=0, atol=1e-9):
            raise ValueError(f'{key} height must agree with CAD fixture and half workpiece height')
    return task


def plan_task(task=None, order=5):
    task = load_task() if task is None else validate_task(task)
    # Begin above the work plane with the tool already oriented for approach.
    # A direct joint-space swing from a horizontal tool would sweep the long
    # jaws through the workpiece before reaching the first waypoint.
    home = np.array(task['pick'], dtype=float)
    home[1] = 0
    home[2] += task['clearance']
    q = inverse(home, task['pitch'])
    opening = task['gripper_open']
    phases = []
    def add(name, target, grip, minimum=0.4):
        nonlocal q, opening
        segment = Segment(q, target, order=order, velocity=task['max_velocity'], acceleration=task['max_acceleration'], min_duration=minimum)
        phases.append(Phase(name, segment, opening, grip))
        q, opening = np.array(target), grip
    pick, place = np.array(task['pick']), np.array(task['place'])
    above = np.array([0, 0, task['clearance']])
    pitch = task['pitch']
    add('SETTLE', q, opening, task['dwell'])
    for name, p in [('APPROACH_PICK', pick+above), ('DESCEND_PICK', pick)]:
        add(name, inverse(p, pitch, q), opening)
    add('CLOSE', q, task['gripper_closed'], task['dwell'])
    add('HOLD', q, opening, task['dwell'])
    for name, p in [('LIFT', pick+above), ('TRANSFER', place+above), ('DESCEND_PLACE', place)]:
        add(name, inverse(p, pitch, q), opening)
    add('OPEN', q, task['gripper_open'], task['dwell'])
    add('RELEASE_SETTLE', q, opening, task['dwell'])
    add('RETRACT', inverse(place+above, pitch, q), opening)
    add('FINISH', q, opening, task['dwell'])
    return phases
