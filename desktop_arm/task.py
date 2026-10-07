"""Explicit manipulation state schedule shared by native and ROS execution."""
from dataclasses import dataclass
import json
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
    return json.loads((project_root() / 'config/task.json' if path is None else path).read_text(encoding='utf-8'))


def plan_task(task=None, order=5):
    task = load_task() if task is None else task
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
