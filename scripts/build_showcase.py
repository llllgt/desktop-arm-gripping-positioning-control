"""Build browser data exclusively from committed simulation evidence."""
import csv
import io
import json
import zipfile
from desktop_arm.paths import project_root


def main():
    root=project_root()
    load=lambda p:json.loads((root/p).read_text(encoding='utf-8'))
    envelope=load('results/operating-envelope/summary.json')
    data={'revision':envelope['model_revision'],'date':'2026-10-08',
          'default':load('results/native/summary.json'),'preflight':load('results/assessment/preflight.json'),
          'structure':load('results/operating-envelope/structure_tradeoff.json'),
          'counts':{k:envelope[k] for k in ('passed','failed','rejected')},'cases':[]}
    grid=load('results/assessment/workspace.json')
    data['workspace']={'height_m':grid['height_m'],'pitch_rad':grid['pitch_rad'],
        'reachable_cells':grid['reachable_cells'],
        'cells':[[*c['xyz_m'][:2],c['reachable']] for c in grid['cells']]}
    with zipfile.ZipFile(root/'results/operating-envelope/traces.zip') as archive:
        for case in envelope['cases']:
            # Every case keeps a direct link to its raw archived trace.
            slim={k:case[k] for k in ('case','mass_g','friction','clearance_mm','status','failure_reasons')}
            for key in ('placement_error_mm','maximum_lift_mm','tcp_rmse_mm','simulation_seconds',
                        'peak_joint_torque_nm','measured_pad_friction_range','metal_workpiece_contact_steps',
                        'bilateral_transport_contact_fraction'):
                slim[key]=case.get(key)
            slim['trace']=[]
            if case['status']!='rejected':
                rows=list(csv.DictReader(io.StringIO(archive.read(case['case']+'/trajectory.csv').decode('utf-8'))))
                selected=rows[::max(1,len(rows)//160)]
                if selected[-1] is not rows[-1]:selected.append(rows[-1])
                slim['trace']=[[round(float(r['time']),3),round(float(r['object_z'])*1000,3),
                                round(float(r['left_normal_force_n']),3),round(float(r['right_normal_force_n']),3)] for r in selected]
            data['cases'].append(slim)
    (root/'docs/explorer-data.js').write_text('window.ARM_RESULTS = '+json.dumps(data,ensure_ascii=False,separators=(',',':'))+';\n',encoding='utf-8')
    print(f"Built showcase: {len(data['cases'])} cases from measured JSON/CSV")


if __name__=='__main__':main()
