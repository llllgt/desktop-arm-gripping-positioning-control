"""Reproduce automatic routes, rejected layouts, timing comparisons and sensitivity."""
import argparse
import copy
import json
from pathlib import Path
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from desktop_arm.paths import project_root
from desktop_arm.task import load_task, plan_task
from desktop_arm.planning import run as plan
from desktop_arm.simulation import run as simulate
from desktop_arm.kinematics import forward


def write(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n', encoding='utf-8')


def compact(name, report):
    execution = report.get('execution', {})
    return {'name':name, 'planned':report['success'],
            'completed':bool(execution.get('success') and report.get('observed_opening_within_envelope')),
            'rejection':report.get('rejection'), 'planning_seconds':report['wall_seconds'],
            'expanded_nodes':report['expanded_nodes'], 'checked_edges':report['checked_edges'],
            'waypoints':len(report.get('waypoints_m', [])),
            'planned_transfer_s':report.get('transfer_duration_s'),
            'simulation_s':execution.get('simulation_seconds'),
            'placement_error_mm':execution.get('placement_error_mm'),
            'minimum_clearance_mm':execution.get('minimum_transport_obstacle_clearance_mm'),
            'failure_reasons':execution.get('failure_reasons', [])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    output = args.output
    if output.exists() and any(output.iterdir()):
        parser.error('Choose an empty output directory')
    output.mkdir(parents=True, exist_ok=True)
    root = project_root()
    records, sensitivity, reports = [], [], {}
    names = ['clear', 'central', 'shifted', 'two_obstacles', 'tall_outer', 'cycle_blocked', 'endpoint_blocked', 'restricted_grid']
    for name in names:
        task = load_task(root/f"config/planning/{'central' if name=='restricted_grid' else name}.json")
        settings = None if name!='restricted_grid' else json.loads((root/'config/planning/restricted_settings.json').read_text())
        report = plan(output/name, task, settings=settings, execute=True, render=args.render and name=='two_obstacles')
        reports[name] = report; records.append(compact(name, report))
        print(name, records[-1], flush=True)
    for resolution in (.01, .02, .03):
        for margin in (.001, .003, .005):
            name = f'grid{round(resolution*1000):02}_margin{round(margin*1000):02}'
            report = plan(output/'sensitivity'/name, load_task(root/'config/planning/central.json'),
                          settings={'resolution_m':resolution, 'obstacle_margin_m':margin}, execute=True)
            sensitivity.append({'resolution_mm':resolution*1000, 'margin_mm':margin*1000, **compact(name, report)})
            print(name, sensitivity[-1], flush=True)
    comparisons = []
    central = load_task(root/'config/planning/central.json')
    raw = reports['central']['raw_joint_path_rad']
    raw_task = copy.deepcopy(central)
    raw_task['transfer_waypoints'] = [forward(q).position.tolist() for q in raw[1:-1]]
    for name, task in [('raw_astar', raw_task),
                       ('manual_side', load_task(root/'config/tasks/side_detour.json')),
                       ('manual_over', load_task(root/'config/tasks/over_barrier.json'))]:
        folder = output/'comparisons'/name; folder.mkdir(parents=True)
        write(folder/'config.json', task)
        result = simulate(folder, task=task, measure_clearance=True)
        phases = plan_task(task)
        transfer = sum(p.segment.duration for p in phases if p.name=='TRANSFER' or p.name.startswith('TRANSFER_VIA_'))
        comparisons.append({'name':name, 'planned_transfer_s':transfer, **result})
    summary = {'scenarios':records, 'sensitivity':sensitivity, 'comparisons':comparisons,
               'scope':'One deterministic simulation per configuration. Search wall time is platform dependent; '
               'sampling, model geometry and grasp assumptions do not establish hardware safety.'}
    write(output/'summary.json', summary)
    plot(output, summary)
    with zipfile.ZipFile(output/'traces.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(output.rglob('*')):
            if path.is_file() and path.suffix in ('.json', '.csv', '.log'):
                archive.write(path, path.relative_to(output).as_posix())


def box(ax, position, size):
    x, y, z = np.asarray(position)*1000
    sx, sy, sz = np.asarray(size)*500
    for height in (z-sz, z+sz):
        ax.plot([x-sx,x+sx,x+sx,x-sx,x-sx], [y-sy,y-sy,y+sy,y+sy,y-sy], [height]*5, color='0.6', lw=.7)
    for a in (-sx,sx):
        for b in (-sy,sy):
            ax.plot([x+a]*2, [y+b]*2, [z-sz,z+sz], color='0.6', lw=.7)


def plot(output, summary):
    fig = plt.figure(figsize=(16, 8), layout='constrained')
    for i, record in enumerate(summary['scenarios']):
        ax = fig.add_subplot(2, 4, i+1, projection='3d')
        folder = output/record['name']
        task = load_task(folder/'request.json')
        for obstacle in task.get('obstacles', []):
            box(ax, obstacle['position'], obstacle['size'])
        points = np.array([task['pick'], task['place']])*1000
        ax.scatter(*points.T, marker='x', color='black', s=18)
        if (folder/'execution/trajectory.csv').exists():
            data = np.genfromtxt(folder/'execution/trajectory.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
            ax.plot(*[data['object_'+a]*1000 for a in 'xyz'], lw=1.5, color='tab:blue')
            ax.set_title(f"{record['name']} | {record['simulation_s']:.2f} s\nerror {record['placement_error_mm']:.3f} mm", fontsize=10)
        else:
            ax.set_title(record['name']+'\nrejected before execution', fontsize=10)
        ax.set_xlim(70,245); ax.set_ylim(-85,85); ax.set_zlim(0,185)
        ax.set_xlabel('x / mm'); ax.set_ylabel('y / mm'); ax.set_zlabel('z / mm')
        ax.view_init(elev=24, azim=-58); ax.tick_params(labelsize=7)
    fig.savefig(output/'automatic_paths.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), layout='constrained')
    for ax, field, title in zip(axes, ['planning_seconds','simulation_s','minimum_clearance_mm'],
                                ['Planning time / s','Simulation cycle / s','Measured obstacle clearance / mm']):
        matrix = np.full((3,3), np.nan)
        for record in summary['sensitivity']:
            i = [10,20,30].index(round(record['resolution_mm']))
            j = [1,3,5].index(round(record['margin_mm']))
            if record[field] is not None:
                matrix[i,j] = record[field]
        shown = ax.imshow(matrix, cmap='Blues')
        for i in range(3):
            for j in range(3):
                color = 'white' if np.isfinite(matrix[i,j]) and shown.norm(matrix[i,j])>.55 else 'black'
                ax.text(j, i, 'reject' if np.isnan(matrix[i,j]) else f'{matrix[i,j]:.2f}', ha='center', va='center', color=color)
        ax.set_xticks(range(3), [1,3,5]); ax.set_yticks(range(3), [10,20,30])
        ax.set_xlabel('Obstacle expansion / mm'); ax.set_ylabel('Grid spacing / mm'); ax.set_title(title, fontsize=10)
        fig.colorbar(shown, ax=ax, shrink=.75)
    fig.savefig(output/'sensitivity.png', dpi=160); plt.close(fig)


if __name__ == '__main__':
    main()
