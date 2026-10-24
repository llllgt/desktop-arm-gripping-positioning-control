"""Compare stopping and continuous trajectories under the same joint limits."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import zipfile
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from desktop_arm.task import load_task
from desktop_arm.motion_experiment import run_motion


def cases():
    root=Path(__file__).resolve().parents[1]
    for name in ('spatial_eight','rising_helix','multi_corner'):
        for mode,ff,label in [('stop',True,'stop'),('continuous',False,'continuous'),('continuous',True,'continuous_ff')]:
            task=load_task(root/'config/continuous'/f'{name}.json')
            task['curve_transfer'].update(mode=mode,velocity_feedforward=ff)
            yield name+'_'+label, name, label, task
    for name in ('spatial_eight','rising_helix','multi_corner'):
        task=load_task(root/'config/continuous'/f'{name}.json')
        variant='chord' if name=='multi_corner' else 'uniform'
        task['curve_transfer']['parameterization']=variant
        yield name+'_'+variant, 'parameterization', variant, task
    for jerk in (2.,30.):
        task=load_task(root/'config/continuous/multi_corner.json')
        task['curve_transfer']['jerk_limit']=jerk
        yield f'corner_jerk{jerk:g}', 'jerk_sensitivity', str(jerk), task
    task=load_task(root/'config/continuous/multi_corner.json')
    task['obstacles']=[{'position':[.12,0,.13],'size':[.025,.025,.06]}]
    yield 'blocked_curve','rejection','obstacle',task
    task=deepcopy(task);task.pop('obstacles')
    task['curve_transfer']['poses'][5][0]=.5
    yield 'unreachable_curve','rejection','reach',task


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--render',action='store_true')
    args=p.parse_args()
    if args.output.exists() and any(args.output.iterdir()):p.error('Output directory must be empty')
    records=[]
    for name,family,variant,task in cases():
        report=run_motion(args.output/name,task,render=args.render and name in ('spatial_eight_continuous_ff','rising_helix_continuous_ff'))
        records.append(dict(name=name,family=family,variant=variant,task=task,report=report))
        print(name,report['success'],report.get('plan',{}).get('transfer_duration_s'),report.get('tracking',{}).get('tcp_tracking_rms_mm'),report.get('rejection'),flush=True)
    (args.output/'summary.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    from plot_continuous import plots
    plots(args.output,records)
    with zipfile.ZipFile(args.output/'traces.zip','w',zipfile.ZIP_DEFLATED) as z:
        for file in sorted(args.output.rglob('*')):
            if file.is_file() and file.suffix in ('.json','.csv'):
                z.write(file,file.relative_to(args.output).as_posix())


if __name__=='__main__':main()
