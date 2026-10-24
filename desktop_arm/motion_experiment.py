"""Run a curved transfer and measure tracking separately from contour deviation."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from .continuous import build_curve
from .kinematics import forward, jacobian
from .task import validate_task


def analyze_trace(path, curve):
    with Path(path).open(encoding='utf-8') as f:
        rows = [r for r in csv.DictReader(f) if r['phase']=='TRANSFER']
    if not rows:
        raise ValueError('No TRANSFER samples in execution trace')
    actual = np.array([[float(r['tcp_'+a]) for a in 'xyz'] for r in rows])
    command = np.array([[float(r['target_'+a]) for a in 'xyz'] for r in rows])
    q = np.array([[float(r[f'q{i}']) for i in range(1,5)] for r in rows])
    qref = np.array([[float(r[f'q{i}_command']) for i in range(1,5)] for r in rows])
    dq = np.array([[float(r[f'dq{i}']) for i in range(1,5)] for r in rows])
    reference = curve.reference(np.linspace(0,1,20001))[:,:3]
    tree = cKDTree(reference)
    command_distance = tree.query(command)[0]
    actual_distance = tree.query(actual)[0]
    tracking = np.linalg.norm(actual-command,axis=1)
    speed = np.array([np.linalg.norm(jacobian(a)@b) for a,b in zip(q,dq)])
    pitch_error = np.sum(q[:,1:]-qref[:,1:],axis=1)
    return dict(samples=len(rows),
                tcp_tracking_rms_mm=float(np.sqrt(np.mean(tracking**2))*1000),
                tcp_tracking_max_mm=float(tracking.max()*1000),
                actual_contour_rms_mm=float(np.sqrt(np.mean(actual_distance**2))*1000),
                command_contour_rms_mm=float(np.sqrt(np.mean(command_distance**2))*1000),
                pitch_tracking_rms_deg=float(np.rad2deg(np.sqrt(np.mean(pitch_error**2)))),
                peak_actual_tcp_speed_mm_s=float(speed.max()*1000),
                peak_actual_joint_velocity_rad_s=np.abs(dq).max(axis=0).tolist(),
                pitch_command_range_deg=[float(np.rad2deg(np.sum(qref[:,1:],axis=1).min())),float(np.rad2deg(np.sum(qref[:,1:],axis=1).max()))],
                reference_extent_xyz_mm=(np.ptp(reference,axis=0)*1000).tolist(),
                contour_method='Nearest of 20001 ideal spline points; geometric distance, not time-indexed error')


def run_motion(output, task, *, render=False):
    from .assessment import preflight
    from .simulation import run
    task = validate_task(task)
    if 'curve_transfer' not in task:
        raise ValueError('Motion experiments require curve_transfer')
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError('Choose an empty output directory')
    output.mkdir(parents=True,exist_ok=True)
    (output/'config.json').write_text(json.dumps(task,indent=2),encoding='utf-8')
    try:
        curve = build_curve(task)
    except ValueError as error:
        report = dict(success=False, rejection=str(error))
        (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        return report
    report = dict(plan=curve.report(), velocity_feedforward=task['curve_transfer'].get('velocity_feedforward',True))
    report['preflight'] = preflight(task)
    if not report['preflight']['feasible']:
        report.update(success=False,rejection=report['preflight']['rejection'])
    else:
        report['execution'] = run(output/'execution',task=task,render=render)
        report['tracking'] = analyze_trace(output/'execution/trajectory.csv',curve)
        report['success'] = report['execution']['success']
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report
