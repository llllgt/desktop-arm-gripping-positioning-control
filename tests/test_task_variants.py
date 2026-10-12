import copy
import numpy as np
import pytest
from desktop_arm.paths import project_root
from desktop_arm.task import load_task, plan_task, validate_task
from desktop_arm.kinematics import forward
from desktop_arm.scene import load_model
from desktop_arm.assessment import preflight
from desktop_arm.simulation import run


def task(name):
    return load_task(project_root()/f'config/tasks/{name}.json')


@pytest.mark.parametrize('name,height', [('higher', .085), ('lower', .035)])
def test_height_variant_scene_and_real_placement(name, height, tmp_path):
    config = task(name)
    model = load_model(config)
    nest = model.geom('place_nest')
    assert abs(nest.pos[2]+nest.size[2]-height)<1e-12
    assert preflight(config)['feasible']
    result = run(tmp_path, task=config)
    assert result['success'] and result['placement_error_mm']<3


def test_direct_barrier_rejected_and_routes_have_real_contact_clearance(tmp_path):
    direct = preflight(task('blocked_direct'))
    assert not direct['feasible']
    assert any('obstacle_1' in c['geoms'] for c in direct['collisions'])
    for name in ('side_detour', 'over_barrier'):
        config = task(name)
        assert preflight(config)['feasible']
        result = run(tmp_path/name, task=config)
        assert result['success']
        assert result['object_obstacle_contact_steps']==0
        assert result['unwanted_contact_steps']==0
        phases = plan_task(config)
        via = [p for p in phases if p.name.startswith('TRANSFER_VIA_')]
        for phase, point in zip(via, config['transfer_waypoints']):
            np.testing.assert_allclose(forward(phase.segment.end).position, point, atol=1e-9)
        for a, b in zip(phases, phases[1:]):
            np.testing.assert_array_equal(a.segment.end, b.segment.start)


def test_carried_box_barrier_collision_is_checked(tmp_path):
    config = task('blocked_direct')
    # Thin short box intersects the carried workpiece even if jaws miss it.
    config['obstacles']=[{'position':[.18, .005, .104], 'size':[.003, .012, .003]}]
    report = preflight(config)
    assert not report['feasible']
    assert any('workpiece_geom' in c['geoms'] for c in report['collisions'])
    result = run(tmp_path, task=config)
    assert result['object_obstacle_contact_steps']>0
    assert 'object_obstacle_collision' in result['failure_reasons']


@pytest.mark.parametrize('key,value', [
    ('fixture_heights', {'place':.006}), ('fixture_heights', {'place':.085}),
    ('fixture_heights', {'unknown':.055}), ('home',[.1,float('nan'),.1]),
    ('transfer_waypoints', [[.1,.1]]), ('transfer_waypoints', [[.1,.1,float('inf')]]),
    ('obstacles',[{'position':[.1,0,.01], 'size':[.02,.02,.04]}]),
    ('obstacles',[{'position':[.1,0,.01], 'size':[.02,0,.02]}]),
])
def test_extension_parameters_reject_invalid_geometry(key,value):
    config = copy.deepcopy(load_task()); config[key]=value
    with pytest.raises(ValueError):
        validate_task(config)
