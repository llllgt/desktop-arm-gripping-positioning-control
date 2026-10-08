"""Deterministic factorial experiment; preserve failed/rejected runs and raw traces."""
import argparse
import itertools
import json
from pathlib import Path
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from desktop_arm.task import load_task
from desktop_arm.assessment import preflight, run as assess
from desktop_arm.simulation import run
from desktop_arm.paths import project_root

MASSES = [.02, .05, .12, .20]
FRICTIONS = [.05, .2, .6, 1.2]
CLEARANCES = [.02, .04, .06]


def structure_tradeoff(output):
    design=json.loads((project_root()/'cad/design.json').read_text(encoding='utf-8'))
    c=design['carrier'];material=design['carrier_material']
    L,b=c['length']/1000,c['width']/1000
    E,F=material['young_modulus_pa'],design['design_load_n']
    cases=[]
    for t_mm in [1,1.5,2,2.5,3]:
        t=t_mm/1000
        stress=6*F*L/(b*t*t)
        cases.append({'thickness_mm':t_mm,'carrier_mass_g':L*b*t*c['density_kg_m3']*1000,
            'beam_deflection_mm':F*L**3/(3*E*(b*t**3/12))*1000,
            'nominal_root_stress_mpa':stress/1e6,'nominal_yield_factor':material['yield_strength_pa']/stress})
    report={'load_n':F,'length_mm':c['length'],'width_mm':c['width'],'cases':cases,
        'scope':'Euler-Bernoulli cantilever comparison only. Full short-edge clamp, distributed tip load; '
        'not a changed CAD assembly or a measured payload capacity. Adhesive and TPU excluded.'}
    (output/'structure_tradeoff.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    axes[0].plot([r['carrier_mass_g'] for r in cases],[r['beam_deflection_mm'] for r in cases],'o-',color='#168e89')
    for r in cases:
        axes[0].annotate(f"{r['thickness_mm']} mm",(r['carrier_mass_g'],r['beam_deflection_mm']),xytext=(3,5),textcoords='offset points')
    axes[0].set(xlabel='Single carrier mass / g',ylabel='Beam tip deflection / mm',title='Thickness tradeoff at 6 N (analytical)')
    axes[1].bar([str(r['thickness_mm']) for r in cases],[r['nominal_root_stress_mpa'] for r in cases],color='#168e89')
    axes[1].set(xlabel='Thickness / mm',ylabel='Nominal root stress / MPa',title='Assumed aluminium, fixed 26 x 14 mm')
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(output/'structure_tradeoff.png',dpi=160);plt.close(fig)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    # Geometry depends on clearance but not on mass or friction.
    checks={}
    for clearance in CLEARANCES:
        task=load_task();task['clearance']=clearance
        checks[clearance]=preflight(task)
    cases=[]
    with zipfile.ZipFile(out/'traces.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for mass,friction,clearance in itertools.product(MASSES,FRICTIONS,CLEARANCES):
            name=f'm{round(mass*1000):03d}_mu{friction:.2f}_h{round(clearance*1000):02d}'
            task=load_task();task.update(workpiece_mass=mass,contact_friction=friction,clearance=clearance)
            folder=out/name;folder.mkdir(exist_ok=True)
            (folder/'task.json').write_text(json.dumps(task,indent=2),encoding='utf-8')
            case={'case':name,'mass_g':mass*1000,'friction':friction,'clearance_mm':clearance*1000,
                  'preflight':checks[clearance]}
            if not checks[clearance]['feasible']:
                case.update(success=False,status='rejected',failure_reasons=['preflight_rejection'])
            else:
                result=run(folder,task=task)
                case.update(result,status='passed' if result['success'] else 'failed')
            cases.append(case)
            (folder/'summary.json').write_text(json.dumps(case,indent=2),encoding='utf-8')
            for file in sorted(folder.iterdir()):
                archive.write(file,f'{name}/{file.name}')
            print(name,case['status'],round(case.get('placement_error_mm',0),3),flush=True)
    summary={'model_revision':'explicit-pad-friction-v2','masses_g':[m*1000 for m in MASSES],
        'frictions':FRICTIONS,'clearances_mm':[h*1000 for h in CLEARANCES],'cases':cases,
        'passed':sum(c['status']=='passed' for c in cases),'failed':sum(c['status']=='failed' for c in cases),
        'rejected':sum(c['status']=='rejected' for c in cases),
        'success_rule':'Final 3D centre error < 8 mm, maximum lift > 60% of requested clearance, no unwanted fixture/floor contact at any physics step.',
        'scope':'48 deterministic factorial configurations, one run each, centred initial workpiece. '
        'Not a probability of success, hardware payload rating or universal envelope.'}
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for ax,h in zip(axes,CLEARANCES):
        grid=np.full((4,4),np.nan)
        for i,m in enumerate(MASSES):
            for j,mu in enumerate(FRICTIONS):
                c=next(c for c in cases if c['mass_g']==m*1000 and c['friction']==mu and c['clearance_mm']==h*1000)
                grid[i,j]=1 if c['status']=='passed' else (0 if c['status']=='failed' else -1)
                ax.text(j,i,'PASS' if c['status']=='passed' else ('FAIL' if c['status']=='failed' else 'REJECT'),ha='center',va='center',fontsize=9)
        ax.imshow(grid,vmin=-1,vmax=1,cmap=matplotlib.colors.ListedColormap(['#b7c4ce','#f4c0ab','#a0dace']))
        ax.set(xticks=range(4),xticklabels=FRICTIONS,yticks=range(4),yticklabels=[round(m*1000) for m in MASSES],
               xlabel='Measured pad sliding friction',ylabel='Workpiece mass / g',title=f'Clearance {round(h*1000)} mm')
    fig.suptitle('48 simulated configurations; PASS/FAIL are model outcomes, not reliability')
    fig.savefig(out/'envelope.png',dpi=160);plt.close(fig)
    structure_tradeoff(out)
    assess(out/'assessment')
    print(json.dumps({k:summary[k] for k in ('passed','failed','rejected')}),flush=True)


if __name__=='__main__':main()
