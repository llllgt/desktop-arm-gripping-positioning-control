import numpy as np
import pytest
import mujoco
from desktop_arm.kinematics import LIMITS, forward, inverse, jacobian
from desktop_arm.trajectory import Segment
from desktop_arm.scene import build_scene, load_model
from desktop_arm.task import plan_task


def test_fk_agrees_with_independent_mujoco_model():
    model = load_model()
    data = mujoco.MjData(model)
    rng = np.random.default_rng(42)
    for _ in range(100):
        q = rng.uniform(LIMITS[:,0], LIMITS[:,1])
        for i in range(4):
            data.qpos[model.joint(f'joint{i+1}').qposadr[0]] = q[i]
        mujoco.mj_forward(model,data)
        np.testing.assert_allclose(forward(q).position,data.site_xpos[model.site('tcp').id],atol=1e-10)
        np.testing.assert_allclose(forward(q).rotation,data.site_xmat[model.site('tcp').id].reshape(3,3),atol=1e-10)


def test_inverse_roundtrip_and_unreachable_rejection():
    # Forward then inverse targets in the canonical radial workspace.
    rng = np.random.default_rng(8)
    checked = 0
    for _ in range(500):
        q = rng.uniform([-1,-0.5,-0.9,-0.3],[1,0.8,0.9,0.8])
        pose = forward(q)
        recovered = inverse(pose.position,pose.pitch,seed=q)
        np.testing.assert_allclose(forward(recovered).position,pose.position,atol=1e-9)
        assert abs(forward(recovered).pitch-pose.pitch)<1e-9
        checked += 1
    assert checked == 500
    with pytest.raises(ValueError):
        inverse([1,0,0],0)
    with pytest.raises(ValueError):
        forward([0,10,0,0])
    with pytest.raises(ValueError):
        inverse([np.nan,0,0],0)


def test_geometric_jacobian_matches_central_difference():
    q = np.array([0.2,-0.3,0.5,0.2])
    eps=1e-6
    finite = np.column_stack([(forward(q+np.eye(4)[i]*eps).position-forward(q-np.eye(4)[i]*eps).position)/(2*eps) for i in range(4)])
    np.testing.assert_allclose(jacobian(q),finite,atol=1e-9)


@pytest.mark.parametrize('order',[3,5])
def test_retiming_obeys_limits_and_end_conditions(order):
    segment=Segment([0,-0.2,0.1,0],[1,0.5,-0.6,0.7],order=order,velocity=[0.3,0.4,0.5,0.6],acceleration=0.8)
    samples=[segment.sample(t) for t in np.linspace(0,segment.duration,10001)]
    vmax=np.max(np.abs([s.velocity for s in samples]),axis=0)
    amax=np.max(np.abs([s.acceleration for s in samples]),axis=0)
    assert np.all(vmax<=np.array([0.3,0.4,0.5,0.6])+1e-9)
    assert np.all(amax<=0.8+1e-9)
    np.testing.assert_allclose(samples[0].position,segment.start)
    np.testing.assert_allclose(samples[-1].position,segment.end)
    np.testing.assert_allclose(samples[0].velocity,0,atol=1e-12)
    np.testing.assert_allclose(samples[-1].velocity,0,atol=1e-12)
    if order==5:
        np.testing.assert_allclose(samples[0].acceleration,0,atol=1e-12)
        np.testing.assert_allclose(samples[-1].acceleration,0,atol=1e-12)


def test_task_continuity_and_scene_uses_contacts_not_attachments():
    phases=plan_task()
    for a,b in zip(phases,phases[1:]):
        np.testing.assert_array_equal(a.segment.end,b.segment.start)
        assert a.gripper_end==b.gripper_start
    assert 'weld' not in build_scene()
    assert 'workpiece_free' in build_scene()


def test_invalid_limits_are_rejected():
    for value in (0,-1,np.nan,np.inf):
        with pytest.raises(ValueError):
            Segment([0]*4,[0.1]*4,velocity=value)
