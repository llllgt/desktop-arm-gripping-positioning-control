"""Sampled fixed-scene preflight and position/pitch workspace assessment.

Checks commanded geometry, not dynamic safety or global collision avoidance.
"""
import json
from pathlib import Path
import numpy as np
import mujoco
from .kinematics import inverse, jacobian, LIMITS
from .task import load_task, plan_task
from .simulation import Simulator


def preflight(task=None, *, order=5, sample_period=.02):
    if not np.isfinite(sample_period) or sample_period <= 0:
        raise ValueError('sample_period must be positive and finite')
    task = load_task() if task is None else task
    report = {'feasible': False, 'sample_period_s': sample_period, 'samples': 0,
              'collisions': [], 'scope': 'Sampled commanded robot versus fixed fixtures/floor; '
              'excludes object contacts and self collision. Not a continuous or dynamic safety guarantee.'}
    try:
        phases = plan_task(task, order)
        sim = Simulator(task)
    except ValueError as error:
        report['rejection'] = str(error)
        return report
    margins, conditioning = [], []
    fixtures = {'pick_nest', 'place_nest', 'pick_baseplate', 'place_baseplate', 'floor'}
    for phase in phases:
        n = max(1, int(np.ceil(phase.segment.duration/sample_period)))
        phase_collisions = set()
        for t in np.linspace(0, phase.segment.duration, n+1):
            command, gripper = phase.command(t)
            q = command.position
            sim.data.qpos[sim.arm_qpos] = q
            for side in ('left', 'right'):
                sim.data.qpos[sim.model.joint(f'gripper_{side}_joint').qposadr[0]] = gripper
            mujoco.mj_forward(sim.model, sim.data)
            margins.append(float(np.min(np.minimum(q-LIMITS[:,0], LIMITS[:,1]-q))))
            singular = np.linalg.svd(jacobian(q), compute_uv=False)
            conditioning.append(float(singular[0]/max(singular[-1],1e-12)))
            report['samples'] += 1
            for c in sim.data.contact:
                if c.dist >= 0:
                    continue
                ids = (int(c.geom1), int(c.geom2))
                names = tuple(sim.model.geom(g).name for g in ids)
                if not (set(names) & fixtures) or 'workpiece_geom' in names:
                    continue
                # World fixtures and base touching the floor are intentional.
                if any(sim.model.geom_bodyid[g] not in (0, sim.model.body('link1').id) for g in ids):
                    phase_collisions.add(names)
        report['collisions'].extend({'phase':phase.name, 'geoms':list(pair)} for pair in sorted(phase_collisions))
    report.update(feasible=not report['collisions'],
                  minimum_joint_margin_rad=min(margins),
                  maximum_position_jacobian_condition=max(conditioning),
                  planned_duration_s=sum(p.segment.duration for p in phases),
                  phase_durations_s={p.name:p.segment.duration for p in phases})
    if report['collisions']:
        report['rejection'] = 'Commanded robot intersects fixtures/floor at sampled configurations'
    return report


def workspace(task=None):
    """Fixed-height XY IK grid: its cells do NOT encode obstacle clearance."""
    task = load_task() if task is None else task
    rows = []
    for x in np.linspace(.08, .28, 41):
        for y in np.linspace(-.15, .15, 61):
            point = [float(x), float(y), float(task['pick'][2])]
            row = {'xyz_m':point, 'reachable':False}
            try:
                q = inverse(point, task['pitch'])
            except ValueError:
                pass
            else:
                row.update(reachable=True, joint_margin_rad=float(np.min(np.minimum(q-LIMITS[:,0], LIMITS[:,1]-q))))
            rows.append(row)
    return {'pitch_rad':task['pitch'], 'height_m':task['pick'][2],
            'shape':[41,61], 'reachable_cells':sum(r['reachable'] for r in rows),
            'scope':'IK reachability only, fixed height/pitch, one selected elbow branch; no collision check.',
            'cells':rows}


def run(output, task=None, order=5):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    report = preflight(task, order=order)
    (output/'preflight.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (output/'workspace.json').write_text(json.dumps(workspace(task),indent=2),encoding='utf-8')
    return report
