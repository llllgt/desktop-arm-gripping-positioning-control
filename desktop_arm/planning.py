"""Bounded Cartesian lattice A* with IK and sampled joint-space edge checks.

The collision model includes the carried box and the swept opening range of
the explicit jaw plates/pads/carriers. Execution uses the unmodified scene.
"""
import copy
import heapq
import itertools
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .paths import project_root
from .task import validate_task, plan_task
from .kinematics import inverse, forward
from .trajectory import Segment
from .scene import build_scene
from .assessment import preflight


DEFAULT_SETTINGS = {
    'bounds_m': [[.08, .22], [-.08, .08], [.105, .145]],
    'resolution_m': .02,
    'obstacle_margin_m': .003,
    'grasp_opening_allowance_m': .002,
    'joint_step_rad': .02,
    'endpoint_neighbors': 16,
    'max_grid_nodes': 3000,
}


def settings_for(settings=None):
    supplied = {} if settings is None else settings
    if not isinstance(supplied, dict) or set(supplied)-set(DEFAULT_SETTINGS):
        raise ValueError('Unknown planner settings')
    result = copy.deepcopy(DEFAULT_SETTINGS)
    result.update(supplied)
    bounds = np.asarray(result['bounds_m'], dtype=float)
    if bounds.shape != (3, 2) or not np.all(np.isfinite(bounds)) or np.any(bounds[:, 1]<bounds[:, 0]):
        raise ValueError('bounds_m must contain three finite [lower, upper] pairs')
    for key in ('resolution_m', 'joint_step_rad', 'obstacle_margin_m', 'grasp_opening_allowance_m'):
        value = np.asarray(result[key], dtype=float)
        if value.shape != () or not np.isfinite(value) or value<0 or (key in ('resolution_m', 'joint_step_rad') and value==0):
            raise ValueError(f'Invalid {key}')
    for key in ('endpoint_neighbors', 'max_grid_nodes'):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, int) or value<1:
            raise ValueError(f'{key} must be a positive integer')
    if result['max_grid_nodes']>3000:
        raise ValueError('max_grid_nodes cannot exceed 3000')
    count = np.prod(np.floor((bounds[:, 1]-bounds[:, 0])/result['resolution_m']+1e-9)+1)
    if count>result['max_grid_nodes']:
        raise ValueError('Grid exceeds max_grid_nodes; increase resolution or reduce bounds')
    return result


class TransferChecker:
    """Rigid carried-box approximation for geometry only; never used to execute."""
    def __init__(self, task, *, margin=.003, joint_step=.02, opening_allowance=.002):
        self.task = task
        self.joint_step = joint_step
        xml = ET.fromstring(build_scene(task))
        assets = {p.name:p.read_bytes() for p in (project_root()/'assets/upstream/meshes').glob('*.stl')}
        nominal = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'), assets=assets)
        state = mujoco.MjData(nominal)
        pick_q = inverse(task['pick'], task['pitch'])
        state.qpos[[nominal.joint(f'joint{i}').qposadr[0] for i in range(1, 5)]] = pick_q
        mujoco.mj_forward(nominal, state)
        grasp_axis = forward(pick_q).rotation[:, 1]
        half_width = float(np.abs(grasp_axis) @ (np.asarray(task['workpiece_size'])/2))
        pad = nominal.geom('left_pad')
        inner_zero = float((state.geom_xpos[pad.id]-state.site_xpos[nominal.site('tcp').id])@grasp_axis-pad.size[1])
        offset_along_grasp = abs(float(np.asarray(task.get('initial_workpiece_offset', [0, 0, 0]))@grasp_axis))
        nominal_opening = half_width+offset_along_grasp-inner_zero
        if nominal_opening<task['gripper_closed'] or nominal_opening>task['gripper_open']:
            raise ValueError('Box width lies outside the configured grasp opening range')
        upper = min(task['gripper_open'], nominal_opening+opening_allowance)
        self.grasp_range = [task['gripper_closed'], upper]
        travel = (upper-task['gripper_closed'])/2
        self.opening = (upper+task['gripper_closed'])/2
        for geom in xml.findall('.//geom'):
            name = geom.get('name', '')
            if name.startswith('obstacle_'):
                size = np.fromstring(geom.get('size'), sep=' ')+margin
                geom.set('size', ' '.join(map(str, size)))
            elif name in {f'{side}_{part}' for side in ('left', 'right') for part in ('metal_contact', 'pad', 'carrier')}:
                # Slides run along local +/-y. At the midpoint joint value,
                # elongating the boxes covers every opening in the interval.
                size = np.fromstring(geom.get('size'), sep=' ')
                size[1] += travel
                geom.set('size', ' '.join(map(str, size)))
                geom.set('contype', '1'); geom.set('conaffinity', '2')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'), assets=assets)
        self.data = mujoco.MjData(self.model)
        self.arm = [self.model.joint(f'joint{i}').qposadr[0] for i in range(1, 5)]
        for side in ('left', 'right'):
            self.data.qpos[self.model.joint(f'gripper_{side}_joint').qposadr[0]] = self.opening
        self.object_qpos = self.model.joint('workpiece_free').qposadr[0]
        self.object_body = self.model.body('workpiece').id
        self.base = self.model.body('link1').id
        self.pick_rotation = forward(inverse(task['pick'], task['pitch'])).rotation
        self.offset = np.asarray(task.get('initial_workpiece_offset', [0, 0, 0]))
        self.samples = 0
        self.last_collision = None

    def valid(self, q):
        self.samples += 1
        self.data.qpos[self.arm] = q
        pose = forward(q)
        rotation = pose.rotation @ self.pick_rotation.T
        self.data.qpos[self.object_qpos:self.object_qpos+3] = pose.position+rotation@self.offset
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rotation.ravel())
        self.data.qpos[self.object_qpos+3:self.object_qpos+7] = quat
        mujoco.mj_forward(self.model, self.data)
        for contact in self.data.contact:
            if contact.dist>=0:
                continue
            ids = (int(contact.geom1), int(contact.geom2))
            bodies = [int(self.model.geom_bodyid[g]) for g in ids]
            # Check moving arm AND carried box against all fixed supports,
            # floor and obstacles. Robot self-contact is outside this check.
            if 0 in bodies and any(b not in (0, self.base) for b in bodies):
                self.last_collision = [self.model.geom(g).name for g in ids]
                return False
        return True

    def edge(self, a, b):
        # All joints share the same monotone scalar profile, so sampling the
        # straight joint segment checks its geometry independently of timing.
        count = max(1, int(np.ceil(np.max(np.abs(b-a))/self.joint_step)))
        return all(self.valid(a+(b-a)*u) for u in np.linspace(0, 1, count+1))


def plan_transfer(task, settings=None):
    task = copy.deepcopy(validate_task(task))
    settings = settings_for(settings)
    started = time.perf_counter()
    report = {'success':False, 'algorithm':'Cartesian lattice A* + duration-based path shortcutting',
              'settings':settings, 'scope':'Fixed pitch, selected IK branch, bounded finite lattice; '
              'sampled robot/carried-box versus fixed scene, no self-collision or continuous-sweep guarantee.',
              'expanded_nodes':0, 'grid_nodes':0, 'checked_edges':0, 'collision_samples':0}
    # Existing via points are deliberately replaced by this search.
    task.pop('transfer_waypoints', None)
    checker = None
    def finish(reason=None, resolved=None):
        report['wall_seconds'] = time.perf_counter()-started
        report['collision_samples'] = 0 if checker is None else checker.samples
        if reason is not None:
            report['rejection'] = reason
        return resolved, report
    try:
        phases = plan_task(task)
    except ValueError as error:
        return finish(str(error))
    start = next(p.segment.end for p in phases if p.name=='LIFT')
    goal = next(p.segment.end for p in phases if p.name=='TRANSFER')
    try:
        checker = TransferChecker(task, margin=settings['obstacle_margin_m'], joint_step=settings['joint_step_rad'],
                                  opening_allowance=settings['grasp_opening_allowance_m'])
    except ValueError as error:
        return finish(str(error))
    report['grasp_opening_range_m'] = checker.grasp_range
    if not checker.valid(start) or not checker.valid(goal):
        report['collision_geoms'] = checker.last_collision
        return finish('Transfer endpoint intersects fixed scene with the planning envelope')
    def duration(a, b):
        return Segment(a, b, velocity=task['max_velocity'], acceleration=task['max_acceleration']).duration
    report['direct_duration_s'] = duration(start, goal)
    report['checked_edges'] += 1
    direct = checker.edge(start, goal)
    report['direct_feasible'] = direct
    if direct:
        raw = [start, goal]
    else:
        bounds = np.asarray(settings['bounds_m'])
        axes = [np.arange(int(np.floor((hi-lo)/settings['resolution_m']+1e-9))+1)*settings['resolution_m']+lo
                for lo, hi in bounds]
        nodes, grid = [start, goal], {}
        positions = [forward(start).position, forward(goal).position]
        for index in itertools.product(*(range(len(a)) for a in axes)):
            p = np.array([axes[i][j] for i, j in enumerate(index)])
            try:
                q = inverse(p, task['pitch'], seed=start)
            except ValueError:
                continue
            grid[index] = len(nodes); nodes.append(q); positions.append(p)
        report['grid_nodes'] = len(grid)
        indices = {node:index for index, node in grid.items()}
        endpoint_links = {}
        for endpoint in (0, 1):
            distances = np.linalg.norm(np.asarray(positions[2:])-positions[endpoint], axis=1) if len(nodes)>2 else []
            endpoint_links[endpoint] = [int(i)+2 for i in np.argsort(distances, kind='stable')[:settings['endpoint_neighbors']]]
        offsets = [i for i in itertools.product((-1, 0, 1), repeat=3) if i!=(0, 0, 0)]
        edge_cache, node_cache = {}, {0:True, 1:True}
        def neighbors(node):
            if node in (0, 1):
                return endpoint_links[node]
            index = indices[node]
            result = [grid[key] for delta in offsets if (key:=tuple(a+b for a, b in zip(index, delta))) in grid]
            result += [e for e in (0, 1) if node in endpoint_links[e]]
            return result
        def edge_valid(a, b):
            key = tuple(sorted((a, b)))
            if key not in edge_cache:
                for node in key:
                    if node not in node_cache:
                        node_cache[node] = checker.valid(nodes[node])
                report['checked_edges'] += 1
                edge_cache[key] = all(node_cache[n] for n in key) and checker.edge(nodes[a], nodes[b])
            return edge_cache[key]
        def heuristic(node):
            # Velocity-only time bound is admissible for these stop-to-stop edges.
            return float(np.max(1.875*np.abs(nodes[1]-nodes[node])/task['max_velocity']))
        costs, parents, closed = {0:0.0}, {}, set()
        queue = [(heuristic(0), 0)]
        while queue:
            _, node = heapq.heappop(queue)
            if node in closed:
                continue
            closed.add(node); report['expanded_nodes'] += 1
            if node==1:
                break
            for other in neighbors(node):
                candidate = costs[node]+duration(nodes[node], nodes[other])
                if candidate>=costs.get(other, float('inf')) or not edge_valid(node, other):
                    continue
                costs[other] = candidate; parents[other] = node
                heapq.heappush(queue, (candidate+heuristic(other), other))
        if 1 not in closed:
            return finish('No route found in the configured lattice and endpoint connections')
        route = [1]
        while route[-1]!=0:
            route.append(parents[route[-1]])
        raw = [nodes[i] for i in reversed(route)]
    report['raw_joint_path_rad'] = [q.tolist() for q in raw]
    report['raw_transfer_duration_s'] = sum(duration(a, b) for a, b in zip(raw, raw[1:]))
    # DAG shortest path over ordered raw-path nodes. A shortcut is accepted
    # only after checking the whole interpolated joint segment.
    costs, parents = [0.0]+[float('inf')]*(len(raw)-1), {}
    for j in range(1, len(raw)):
        for i in range(j):
            candidate = costs[i]+duration(raw[i], raw[j])
            if candidate>=costs[j]:
                continue
            report['checked_edges'] += 1
            if checker.edge(raw[i], raw[j]):
                costs[j] = candidate; parents[j] = i
    path = [len(raw)-1]
    while path[-1]!=0:
        path.append(parents[path[-1]])
    selected = [raw[i] for i in reversed(path)]
    task['transfer_waypoints'] = [forward(q).position.tolist() for q in selected[1:-1]]
    # Re-resolve IK exactly as the executor will. Reject any branch change that
    # would make the saved Cartesian waypoints execute an unchecked segment.
    resolved_phases = plan_task(task)
    transfer = [p for p in resolved_phases if p.name=='TRANSFER' or p.name.startswith('TRANSFER_VIA_')]
    if not all(checker.edge(p.segment.start, p.segment.end) for p in transfer):
        return finish('Resolved IK route failed the final envelope check')
    full_report = preflight(task)
    report['preflight'] = full_report
    if not full_report['feasible']:
        return finish('Transfer route found, but approach/lift/place/retract preflight failed')
    report.update(success=True, waypoints_m=task['transfer_waypoints'],
                  transfer_duration_s=sum(p.segment.duration for p in transfer),
                  task_duration_s=full_report['planned_duration_s'],
                  joint_path_rad=[transfer[0].segment.start.tolist()]+[p.segment.end.tolist() for p in transfer])
    return finish(resolved=task)


def run(output, task, *, settings=None, execute=False, render=False):
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Choose an empty output directory to preserve earlier plans and traces')
    output.mkdir(parents=True, exist_ok=True)
    resolved, report = plan_transfer(task, settings)
    for name, content in [('request.json', task), ('planning.json', report)]:
        (output/name).write_text(json.dumps(content, indent=2)+'\n', encoding='utf-8')
    if resolved is not None:
        (output/'resolved_task.json').write_text(json.dumps(resolved, indent=2)+'\n', encoding='utf-8')
        if execute:
            from .simulation import run as simulate
            report['execution'] = simulate(output/'execution', task=resolved, render=render, measure_clearance=True)
            bounds = report['grasp_opening_range_m']
            measured = report['execution']['transport_gripper_range_m']
            report['observed_opening_within_envelope'] = all(bounds[0]-1e-6<=lo<=hi<=bounds[1]+1e-6 for lo, hi in measured)
            (output/'planning.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report
