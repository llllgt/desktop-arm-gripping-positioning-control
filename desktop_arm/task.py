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
    for key in ('pick', 'place', 'workpiece_size', 'initial_workpiece_offset', 'home'):
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
    heights = task.get('fixture_heights', {})
    if not isinstance(heights, dict) or set(heights) - {'pick', 'place'}:
        raise ValueError('fixture_heights accepts only pick and place heights in metres')
    for key in ('pick', 'place'):
        height = np.asarray(heights.get(key, design['fixture']['total_height']/1000), dtype=float)
        if height.shape != () or not np.isfinite(height) or height <= design['fixture']['base_thickness']/1000:
            raise ValueError(f'{key} fixture height must exceed base thickness')
        if not np.isclose(task[key][2] - task['workpiece_size'][2]/2,
                          height, rtol=0, atol=1e-9):
            raise ValueError(f'{key} height must agree with CAD fixture and half workpiece height')
    waypoints = np.asarray(task.get('transfer_waypoints', []), dtype=float)
    if waypoints.size and (waypoints.ndim != 2 or waypoints.shape[1] != 3 or not np.all(np.isfinite(waypoints))):
        raise ValueError('transfer_waypoints must be a list of finite xyz positions in metres')
    if waypoints.size == 0 and task.get('transfer_waypoints', []) != []:
        raise ValueError('transfer_waypoints must be an empty list or xyz positions')
    obstacles = task.get('obstacles', [])
    if not isinstance(obstacles, list):
        raise ValueError('obstacles must be a list of boxes')
    for obstacle in obstacles:
        if not isinstance(obstacle, dict) or set(obstacle) != {'position', 'size'}:
            raise ValueError('Each obstacle requires position and full size in metres')
        position = np.asarray(obstacle['position'], dtype=float)
        size = np.asarray(obstacle['size'], dtype=float)
        if position.shape != (3,) or size.shape != (3,) or not np.all(np.isfinite([position, size])) or np.any(size <= 0):
            raise ValueError('Obstacle position/size must contain three finite values; size must be positive')
        if position[2] < size[2]/2:
            raise ValueError('Obstacle must not extend below the floor')
    return task


def is_transport_phase(name):
    return name in ('LIFT', 'TRANSFER', 'DESCEND_PLACE') or name.startswith('TRANSFER_VIA_')


def plan_task(task=None, order=5):
    task = load_task() if task is None else validate_task(task)
    # Begin above the work plane with the tool already oriented for approach.
    # A direct joint-space swing from a horizontal tool would sweep the long
    # jaws through the workpiece before reaching the first waypoint.
    home = np.array(task['pick'], dtype=float)
    home[1] = 0
    home[2] += task['clearance']
    home = np.asarray(task.get('home', home), dtype=float)
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
    route = [('LIFT', pick+above)]
    route += [(f'TRANSFER_VIA_{i+1}', np.asarray(p)) for i, p in enumerate(task.get('transfer_waypoints', []))]
    route += [('TRANSFER', place+above), ('DESCEND_PLACE', place)]
    for name, p in route:
        add(name, inverse(p, pitch, q), opening)
    add('OPEN', q, task['gripper_open'], task['dwell'])
    add('RELEASE_SETTLE', q, opening, task['dwell'])
    add('RETRACT', inverse(place+above, pitch, q), opening)
    add('FINISH', q, opening, task['dwell'])
    return phases
