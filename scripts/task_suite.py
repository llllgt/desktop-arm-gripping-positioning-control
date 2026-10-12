"""Compare transfer scenarios with sampled preflight, contact execution and traces."""
import argparse
import json
from pathlib import Path
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from desktop_arm.paths import project_root
from desktop_arm.task import load_task, plan_task
from desktop_arm.kinematics import forward
from desktop_arm.assessment import preflight
from desktop_arm.simulation import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', choices=['side_detour', 'over_barrier'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    root = project_root()
    configs = [('baseline', root/'config/task.json')]
    configs += [(p.stem, p) for p in sorted((root/'config/tasks').glob('*.json'))]
    records = []
    for name, path in configs:
        task = load_task(path)
        folder = args.output/name
        folder.mkdir(exist_ok=True)
        report = preflight(task)
        (folder/'config.json').write_text(json.dumps(task, indent=2)+'\n', encoding='utf-8')
        (folder/'preflight.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        record = {'name': name, 'config': str(path.relative_to(root)).replace('\\', '/'),
                  'preflight': report}
        if report['feasible']:
            record['simulation'] = run(folder, task=task, render=name==args.render)
        records.append(record)
        print(name, 'rejected' if not report['feasible'] else record['simulation']['success'], flush=True)
    summary = {'scenarios': records, 'executed': sum('simulation' in r for r in records),
               'completed': sum(r.get('simulation', {}).get('success', False) for r in records),
               'preflight_rejected': sum(not r['preflight']['feasible'] for r in records),
               'scope': 'One deterministic run per executable scenario, same 30 g box and contact model.'}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    plot(args.output, records)
    with zipfile.ZipFile(args.output/'traces.zip', 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for folder in [args.output/r['name'] for r in records]:
            for path in sorted(folder.glob('*')):
                if path.suffix in ('.csv', '.json'):
                    archive.write(path, path.relative_to(args.output).as_posix())


def plot(output, records):
    fig, axes = plt.subplots(2, 4, figsize=(15, 7), layout='constrained')
    for ax, record in zip(axes.flat, records):
        task = load_task(project_root()/record['config'])
        ax.scatter(*np.array([task['pick'], task['place']])[:, :2].T*1000,
                   marker='x', color='black', label='stations')
        for obstacle in task.get('obstacles', []):
            p, s = np.array(obstacle['position'])*1000, np.array(obstacle['size'])*1000
            ax.add_patch(Rectangle(p[:2]-s[:2]/2, *s[:2], facecolor='0.8', edgecolor='0.3'))
        if 'simulation' in record:
            data = np.genfromtxt(output/record['name']/'trajectory.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
            ax.plot(data['object_x']*1000, data['object_y']*1000, lw=1.3, label='measured box')
            ax.plot(data['target_x']*1000, data['target_y']*1000, '--', lw=1, label='commanded TCP')
            result = record['simulation']
            ax.set_title(f"{record['name']} | {result['placement_error_mm']:.3f} mm\n{result['simulation_seconds']:.3f} s")
        else:
            points = np.array([forward(p.segment.sample(t).position).position
                               for p in plan_task(task) for t in np.linspace(0, p.segment.duration, 80)])
            ax.plot(*points[:, :2].T*1000, '--', color='tab:red')
            ax.set_title(record['name']+' | rejected before execution')
        ax.title.set_fontsize(10)
        ax.set_xlim(75, 225); ax.set_ylim(-85, 95)
        ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set_xlabel('x / mm'); ax.set_ylabel('y / mm')
    axes.flat[0].legend(fontsize=7)
    fig.savefig(output/'paths.png', dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 4), layout='constrained')
    for name in ('baseline', 'lower', 'higher', 'side_detour', 'over_barrier'):
        data = np.genfromtxt(output/name/'trajectory.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
        ax.plot(data['time'], data['object_z']*1000, label=name)
    ax.set_xlabel('simulation time / s'); ax.set_ylabel('measured box centre z / mm')
    ax.legend(ncol=3); ax.grid(alpha=.2)
    fig.savefig(output/'heights.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
