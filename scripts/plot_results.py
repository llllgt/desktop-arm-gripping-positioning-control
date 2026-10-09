"""Plot measured outputs; never synthesize performance data."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]


def main():
    rows=list(csv.DictReader((ROOT/'results/native/trajectory.csv').open(encoding='utf-8')))
    values=lambda key:np.array([float(row[key]) for row in rows])
    t=values('time')
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for i in range(1,5):
        axes[0,0].plot(t,(values(f'q{i}')-values(f'q{i}_command'))*180/np.pi,label=f'Joint {i}')
        axes[1,0].plot(t,values(f'tau{i}'),label=f'Joint {i}')
    axes[0,0].set_ylabel('Tracking error / deg');axes[0,0].legend(ncol=2)
    axes[1,0].set_ylabel('Total actuator torque / Nm');axes[1,0].axhline(2,ls='--',c='grey');axes[1,0].axhline(-2,ls='--',c='grey')
    for axis in 'xyz':
        axes[0,1].plot(t,values('object_'+axis)*1000,label=axis)
    axes[0,1].set_ylabel('Workpiece position / mm');axes[0,1].legend()
    bench=json.loads((ROOT/'results/benchmark/summary.json').read_text())
    cases=[case for case in bench['cases'] if case['case'].startswith('perturbation_')]
    # Read case summaries as well if the aggregate uses a separate perturbations list.
    if not cases:
        cases=[json.loads(p.read_text()) for p in sorted((ROOT/'results/benchmark').glob('perturbation_*/summary.json'))]
    axes[1,1].bar(np.arange(1,len(cases)+1),[v['placement_error_mm'] for v in cases],color='#168e89')
    axes[1,1].set_xlabel('Perturbation trial');axes[1,1].set_ylabel('Placement error / mm')
    axes[1,1].set_title('10 perturbed task configurations')
    for ax in axes.ravel():
        ax.grid(alpha=.2)
    for ax in [axes[0,0],axes[0,1],axes[1,0]]:
        ax.set_xlabel('Simulation time / s')
    fig.suptitle('Four-DOF arm: pick-and-place simulation')
    fig.savefig(ROOT/'results/native/performance.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    main()
