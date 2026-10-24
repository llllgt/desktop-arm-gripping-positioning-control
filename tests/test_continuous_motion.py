from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from scipy.interpolate import PPoly
from desktop_arm.action_trajectory import ActionTrajectory
from desktop_arm.continuous import build_curve, polynomial_bounds
from desktop_arm.motion_experiment import run_motion
from desktop_arm.task import load_task, plan_task
from desktop_arm.kinematics import forward
from desktop_arm.paths import project_root


def task(name='multi_corner'):
    return load_task(project_root()/'config/continuous'/f'{name}.json')


@pytest.mark.parametrize('name',['spatial_eight','rising_helix','multi_corner'])
def test_continuous_curve_derivatives_limits_and_geometry(name):
    c=build_curve(task(name))
    points=np.linspace(0,c.duration,5001)
    samples=[c.sample(t) for t in points]
    for key,limit in [('velocity',.8),('acceleration',1.5),('jerk',8.)]:
        assert np.max(np.abs([getattr(s,key) for s in samples]))<=limit+1e-7
        np.testing.assert_allclose(getattr(samples[0],key),0,atol=1e-12)
        np.testing.assert_allclose(getattr(samples[-1],key),0,atol=1e-12)
    # Inspect random points, not the IK fitting nodes or their midpoints.
    for u in np.random.default_rng(4).uniform(.01,.99,45):
        q=c.sample(u*c.duration).position
        assert np.linalg.norm(forward(q).position-c.reference(u)[:3])<1e-6
    assert min(np.linalg.norm(c.sample(u*c.duration).velocity) for u in c.nodes[1:-1])>1e-5
    # Analytical derivatives checked against independent time differences.
    t=.37*c.duration;h=.0001
    plus,minus,mid=c.sample(t+h),c.sample(t-h),c.sample(t)
    np.testing.assert_allclose((plus.position-minus.position)/(2*h),mid.velocity,atol=1e-6)
    np.testing.assert_allclose((plus.velocity-minus.velocity)/(2*h),mid.acceleration,atol=1e-5)
    np.testing.assert_allclose((plus.acceleration-minus.acceleration)/(2*h),mid.jerk,atol=1e-4)


def test_polynomial_extrema_finds_between_sample_peak():
    # p(t)=4*t*(1-t), zero at both endpoints but maximum 1 at t=.5.
    p=PPoly(np.array([[-4.],[4.],[0.]]),[0.,1.])
    low,high=polynomial_bounds([p,p,p,p])
    np.testing.assert_allclose(high[:,0],1)
    np.testing.assert_allclose(low[:,2],-8)


def test_stop_baseline_and_tighter_jerk():
    t=task();t['curve_transfer']['mode']='stop';c=build_curve(t)
    for time in c.stops:
        np.testing.assert_allclose(c.sample(time).velocity,0,atol=1e-10)
    t=task();normal=build_curve(t)
    t['curve_transfer']['jerk_limit']=.5;slow=build_curve(t)
    assert slow.duration>normal.duration
    assert np.max(slow.peaks[:,2])<.5
    for a,b in zip(plan_task(t),plan_task(t)[1:]):
        np.testing.assert_allclose(a.segment.end,b.segment.start,atol=1e-12)


@pytest.mark.parametrize('mutation',[
    lambda t:t['curve_transfer'].update(jerk_limit=0),
    lambda t:t['curve_transfer'].update(ik_samples=30),
    lambda t:t['curve_transfer']['poses'][0].__setitem__(0,.18),
    lambda t:t.update(transfer_waypoints=[[.14,0,.12]]),
    lambda t:t['curve_transfer'].update(parameterization='unknown'),
])
def test_bad_curve_inputs_rejected(mutation):
    t=task();mutation(t)
    with pytest.raises(ValueError):build_curve(t)


def test_action_uses_derivatives_and_rejects_internal_overspeed():
    c=build_curve(task());times=np.linspace(0,c.duration,int(np.ceil(c.duration/.02))+1)
    samples=[c.sample(t) for t in times]
    q,v,a=[np.array([getattr(s,key) for s in samples]) for key in ['position','velocity','acceleration']]
    action=ActionTrajectory(times,q,v,a,velocity=.8,acceleration=1.5,jerk=8.)
    for t in np.random.default_rng(2).uniform(0,c.duration,100):
        np.testing.assert_allclose(action.sample(t).position,c.sample(t).position,atol=1e-8)
        np.testing.assert_allclose(action.sample(t).velocity,c.sample(t).velocity,atol=1e-6)
    with pytest.raises(ValueError,match='exceeds'):
        ActionTrajectory([0,1],[[0]*4,[.1,0,0,0]],np.zeros((2,4)),np.zeros((2,4)),velocity=.15,acceleration=10,jerk=100)
    with pytest.raises(ValueError,match='require'):
        ActionTrajectory([0,1],np.zeros((2,4)),[[],[]],[[],[]],velocity=1,acceleration=1,jerk=1)


def test_feedforward_tracking_and_real_contact_transfer(tmp_path):
    t=task();t['curve_transfer']['velocity_feedforward']=False
    off=run_motion(tmp_path/'off',t)
    t['curve_transfer']['velocity_feedforward']=True
    on=run_motion(tmp_path/'on',t)
    assert on['success'] and off['success']
    assert on['tracking']['tcp_tracking_rms_mm'] < .4*off['tracking']['tcp_tracking_rms_mm']
    assert not on['execution']['object_attachment_used']
    assert max(on['execution']['peak_joint_torque_nm'])<2.00001
    with pytest.raises(ValueError,match='empty'):run_motion(tmp_path/'on',t)


def test_unreachable_curve_preserves_rejection(tmp_path):
    t=task();t['curve_transfer']['poses'][5][0]=.5
    r=run_motion(tmp_path/'rejected',t)
    assert not r['success'] and 'rejection' in r
    assert not (tmp_path/'rejected/execution').exists()
