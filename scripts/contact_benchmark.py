"""Reproduce position/admittance contrasts, spring sensitivity and fault cases."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import zipfile
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop_arm.contact_process import run_contact
from desktop_arm.force_control import ContactConfig


def cases():
    for stiffness in (400, 800, 1600):
        for offset in (-0.002, 0, 0.002):
            for mode in ('position', 'admittance'):
                name = f'k{stiffness}_height{round(offset*1000):+d}_{mode}'
                yield name, 'height_stiffness', ContactConfig(mode=mode, stiffness=stiffness,
                                                            surface_offset=offset, surface_tilt_deg=0)
    for mode in ('position', 'admittance'):
        yield 'tilt_'+mode, 'tilt', ContactConfig(mode=mode)
    yield 'fast_scan', 'speed', ContactConfig(surface_offset=.002, surface_tilt_deg=0, scan_seconds=2)
    yield 'noise_delay', 'sensor', ContactConfig(sensor_noise=0.15, sensor_delay=0.06)
    yield 'sensor_bias', 'sensor', ContactConfig(sensor_bias=0.45)
    yield 'missing_surface', 'fault', ContactConfig(surface_present=False)
    yield 'overload', 'fault', ContactConfig(mode='position', surface_offset=0.006, force_limit=2.7)
    yield 'stroke_limit', 'fault', ContactConfig(stiffness=300, target_force=4.0)


def plots(output, records):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout='constrained')
    for name, label, color in [('tilt_position','Fixed height','#ba5a31'), ('tilt_admittance','Force feedback','#146c94')]:
        d = np.genfromtxt(output/name/'trace.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
        scan = d[d['phase']=='SCAN']; t = scan['time']-scan['time'][0]
        axes[0,0].plot(t,scan['force_z_true'],label=label,color=color)
        axes[0,1].plot(scan['tcp_x']*1000,scan['tcp_z']*1000,label=label,color=color)
        axes[1,0].plot(t,scan['compression_m']*1000,label=label,color=color)
    axes[0,0].axhline(2,color='black',ls='--',lw=1,label='2 N target')
    axes[0,0].set(xlabel='Scan time / s',ylabel='Actual contact force Z / N',title='Tilted plate: -3 degrees')
    axes[0,0].legend(fontsize=9)
    axes[0,1].set(xlabel='Measured TCP x / mm',ylabel='Measured TCP z / mm',title='Height adjustment during scan')
    axes[1,0].set(xlabel='Scan time / s',ylabel='Probe compression / mm',title='Passive spring deflection')
    ax=axes[1,1]
    for mode,color in [('position','#ba5a31'),('admittance','#146c94')]:
        rows=[r for r in records if r['group']=='height_stiffness' and r['config']['mode']==mode]
        x=np.arange(len(rows))
        ax.plot(x,[r['summary']['scan_force_rmse_n'] if r['summary']['scan_force_rmse_n'] is not None else np.nan for r in rows],'o-',color=color,label=mode)
    ax.set_xticks(x,[f"{r['config']['stiffness']}\n{r['config']['surface_offset']*1000:+.0f}" for r in rows],fontsize=8)
    ax.axhline(.3,color='gray',ls='--',lw=1)
    ax.set(xlabel='Spring N/m / height error mm',ylabel='Scan force RMSE / N',title='Height uncertainty and spring stiffness')
    ax.legend(fontsize=9)
    fig.savefig(output/'force_comparison.png',dpi=180); plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(12,3.5),layout='constrained')
    for ax,name,title in zip(axes,['missing_surface','overload','stroke_limit'],['No surface','Force threshold','Probe stroke limit']):
        d=np.genfromtxt(output/name/'trace.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
        ax.plot(d['time'],d['force_z_true'],label='Actual force',color='#146c94')
        ax.plot(d['time'],d['sensor_raw'],label='Sensor',color='#ba5a31',alpha=.45,lw=.7)
        idx=np.flatnonzero(d['phase']=='RETRACT')
        if len(idx): ax.axvline(d['time'][idx[0]],ls='--',color='black',lw=1,label='Retract command')
        ax.set(xlabel='Time / s',ylabel='Force / N',title=title)
    axes[0].legend(fontsize=8)
    fig.savefig(output/'fault_response.png',dpi=180);plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--render',action='store_true')
    args=parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Output directory must be empty')
    records=[]
    for name,group,c in cases():
        report=run_contact(args.output/name,c,render=args.render and name=='tilt_admittance')
        records.append(dict(name=name,group=group,config=asdict(c),summary=report))
        print(name,report['success'],report['scan_force_rmse_n'],report['failure_reasons'],flush=True)
    (args.output/'summary.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    plots(args.output,records)
    with zipfile.ZipFile(args.output/'traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for name,_,_ in cases():
            for filename in ('config.json','summary.json','trace.csv'):
                path=args.output/name/filename
                archive.write(path,path.relative_to(args.output).as_posix())


if __name__=='__main__':
    main()
