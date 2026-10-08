import copy
import numpy as np
import pytest
from desktop_arm.task import load_task, validate_task
from desktop_arm.assessment import preflight
from desktop_arm.simulation import run


@pytest.mark.parametrize('key,value',[
    ('workpiece_mass',0), ('clearance',float('nan')), ('pick',[.1,.2]),
    ('place',[.17,.06,.07]), ('contact_friction',-.1), ('max_velocity',[.8]*3),
    ('gripper_closed',.02), ('workpiece_size',[.03,0,.02]),
])
def test_invalid_task_parameters_are_rejected(key,value):
    task=copy.deepcopy(load_task());task[key]=value
    with pytest.raises(ValueError):
        validate_task(task)


def test_preflight_rejects_unreachable_waypoints_and_accepts_default():
    task=load_task()
    result=preflight(task)
    assert result['feasible'] and result['minimum_joint_margin_rad']>0
    assert result['samples']>500
    task['place'][0]=.8
    result=preflight(task)
    assert not result['feasible'] and 'reach' in result['rejection']


def test_preflight_detects_intermediate_fixture_collision():
    task=load_task();task['clearance']=.001
    result=preflight(task,sample_period=.01)
    assert not result['feasible']
    assert result['collisions']


def test_low_friction_reaches_actual_contacts_and_changes_transfer(tmp_path):
    # Regression for old max(geom friction) masking low pad friction.
    task=load_task();task['contact_friction']=.05;task['workpiece_mass']=.12
    result=run(tmp_path,task=task)
    np.testing.assert_allclose(result['measured_pad_friction_range'],[.05,.05])
    assert not result['success']
    assert result['metal_workpiece_contact_steps']==0
    assert max(result['peak_joint_torque_nm'])<=2.000001
