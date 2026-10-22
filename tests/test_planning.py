import itertools
import json
import numpy as np
import mujoco
import pytest
from desktop_arm.paths import project_root
from desktop_arm.task import load_task, plan_task
from desktop_arm.planning import TransferChecker, plan_transfer, settings_for, run
from desktop_arm.scene import load_model


def scene(name):
    return load_task(project_root()/f'config/planning/{name}.json')


def test_edge_collision_between_free_endpoints_is_not_missed():
    task = scene('central')
    phases = plan_task(task)
    phase = next(p for p in phases if p.name=='TRANSFER')
    check = TransferChecker(task)
    assert check.valid(phase.segment.start) and check.valid(phase.segment.end)
    assert not check.edge(phase.segment.start, phase.segment.end)


def test_gripper_envelope_contains_contact_parts_across_opening_interval():
    task = scene('central')
    check = TransferChecker(task)
    q = next(p.segment.end for p in plan_task(task) if p.name=='LIFT')
    check.valid(q)
    model = load_model(task); data = mujoco.MjData(model)
    data.qpos[[model.joint(f'joint{i}').qposadr[0] for i in range(1, 5)]] = q
    signs = np.array(list(itertools.product((-1, 1), repeat=3)))
    for opening in np.linspace(*check.grasp_range, 5):
        for side in ('left', 'right'):
            data.qpos[model.joint(f'gripper_{side}_joint').qposadr[0]] = opening
        mujoco.mj_forward(model, data)
        for side in ('left', 'right'):
            for part in ('pad', 'carrier', 'metal_contact'):
                name = f'{side}_{part}'
                source, envelope = model.geom(name), check.model.geom(name)
                corners = (signs*source.size)@data.geom_xmat[source.id].reshape(3,3).T+data.geom_xpos[source.id]
                local = (corners-check.data.geom_xpos[envelope.id])@check.data.geom_xmat[envelope.id].reshape(3,3)
                assert np.all(np.abs(local)<=envelope.size+1e-10)


def test_search_shortcuts_and_executes_real_contact_transfer(tmp_path):
    report = run(tmp_path, scene('central'), execute=True)
    assert report['success'] and not report['direct_feasible']
    assert report['execution']['success'] and report['observed_opening_within_envelope']
    assert report['execution']['minimum_transport_obstacle_clearance_mm']>0
    assert report['transfer_duration_s']<report['raw_transfer_duration_s']
    assert report['execution']['placement_error_mm']<3
    assert not report['execution']['object_attachment_used']
    resolved = load_task(tmp_path/'resolved_task.json')
    dense = TransferChecker(resolved, joint_step=.005)
    for phase in plan_task(resolved):
        if phase.name=='TRANSFER' or phase.name.startswith('TRANSFER_VIA_'):
            assert dense.edge(phase.segment.start, phase.segment.end)
    # Same scene/settings give the same route; wall time is not deterministic.
    _, again = plan_transfer(scene('central'))
    np.testing.assert_allclose(again['waypoints_m'], report['waypoints_m'], atol=1e-12)


def test_clear_scene_keeps_direct_transfer():
    resolved, report = plan_transfer(scene('clear'))
    assert report['success'] and report['direct_feasible']
    assert resolved['transfer_waypoints']==[]


def test_restricted_grid_fails_without_claiming_physical_impossibility(tmp_path):
    settings = json.loads((project_root()/'config/planning/restricted_settings.json').read_text())
    report = run(tmp_path, scene('central'), settings=settings)
    assert not report['success'] and 'configured lattice' in report['rejection']
    assert not (tmp_path/'resolved_task.json').exists()
    assert not (tmp_path/'execution').exists()
    assert (tmp_path/'planning.json').exists()
    with pytest.raises(FileExistsError):
        run(tmp_path, scene('clear'))


def test_full_cycle_rejects_collision_after_transfer_route_is_found():
    resolved, report = plan_transfer(scene('cycle_blocked'))
    assert resolved is None and not report['success']
    assert report['preflight']['collisions']


@pytest.mark.parametrize('settings', [
    {'resolution_m':0}, {'joint_step_rad':float('nan')}, {'obstacle_margin_m':-.001},
    {'bounds_m':[[.2,.1],[0,1],[0,1]]}, {'resolution_m':.0001},
    {'endpoint_neighbors':True}, {'max_grid_nodes':3001}, {'unknown':1},
])
def test_invalid_and_excessive_search_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        settings_for(settings)
