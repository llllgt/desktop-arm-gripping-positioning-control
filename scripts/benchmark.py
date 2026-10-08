"""Small reproducible ablation/perturbation set, not a real-world reliability claim."""
from pathlib import Path
import json
import argparse
import numpy as np
from desktop_arm.simulation import run
from desktop_arm.task import load_task

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=ROOT/'results/benchmark')
out=parser.parse_args().output;out.mkdir(parents=True,exist_ok=True)
cases=[]
for name,kwargs in [('quintic',{}),('cubic',{'order':3}),('no_feedforward',{'gravity_compensation':False}),('no_contact_pads',{'fingers':False})]:
    cases.append({'case':name,**run(out/name,**kwargs)})
rng=np.random.default_rng(20261007)
for i in range(10):
    task=load_task()
    task['initial_workpiece_offset']=[*rng.uniform(-.002,.002,2),0]
    task['workpiece_mass']=float(rng.uniform(.02,.05))
    task['contact_friction']=float(rng.uniform(.8,1.2))
    result=run(out/f'perturbation_{i:02}',task=task)
    cases.append({'case':f'perturbation_{i:02}','mass_g':task['workpiece_mass']*1000,'friction':task['contact_friction'],'initial_offset_mm':(np.array(task['initial_workpiece_offset'])*1000).tolist(),**result})
summary={'seed':20261007,'cases':cases,'perturbation_successes':sum(c['success'] for c in cases[4:]),'perturbation_trials':10,'scope':'Ten small model perturbations; synthetic simulation only; no statistical reliability or real-world success-rate claim.'}
(out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print([(c['case'],c['success'],round(c['placement_error_mm'],3)) for c in cases])
