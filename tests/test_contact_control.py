import numpy as np
import pytest
from desktop_arm.contact_process import ContactExperiment, ForceSensor, probe_force, run_contact
from desktop_arm.force_control import ContactConfig, ControllerSettings


@pytest.mark.parametrize('kwargs', [
    {'target_force': float('nan')}, {'stiffness': 0}, {'sensor_delay': -.1},
    {'mode': 'unknown'}, {'surface_present': 'false'}, {'target_force': 7},
    {'timestep': .02}, {'scan_seconds': 0},
])
def test_contact_rejects_unsupported_or_nonfinite_parameters(kwargs):
    with pytest.raises(ValueError):
        ContactConfig(**kwargs)


def test_delayed_sensor_and_geometry_separation():
    c = ContactConfig(sensor_noise=0, sensor_delay=.004, sensor_bias=.2)
    sensor = ForceSensor(c)
    assert sensor.sample(1)[0] == 0
    assert sensor.sample(2)[0] == 0
    assert sensor.sample(3)[0] == pytest.approx(1.2)
    settings = ControllerSettings.from_experiment(c)
    assert not hasattr(settings, 'surface_offset')
    assert not hasattr(settings, 'surface_tilt_deg')
    assert not hasattr(settings, 'surface_present')


def test_force_sign_and_static_spring_equilibrium():
    sim = ContactExperiment(ContactConfig(surface_tilt_deg=0, sensor_noise=0, scan_seconds=2))
    for _ in range(2200):
        sim.step()
    force, bad = probe_force(sim.model, sim.data)
    # The probe is supported against gravity by contact. This independent
    # static relation also checks the contact-frame force/sign conversion.
    spring = 800*sim.data.qpos[sim.spring_q] + .018*9.81
    assert force[2] > 1.5
    assert force[2] == pytest.approx(spring, abs=.03)
    assert not bad


def test_feedback_compensates_height_error_and_retracts(tmp_path):
    baseline = run_contact(tmp_path/'position', ContactConfig(mode='position', surface_offset=.002, surface_tilt_deg=0))
    feedback = run_contact(tmp_path/'feedback', ContactConfig(surface_offset=.002, surface_tilt_deg=0))
    assert feedback['success']
    assert feedback['scan_force_rmse_n'] < .1
    assert baseline['scan_force_rmse_n'] > 1
    assert not baseline['success']
    assert feedback['final_force_n'] < .1
    assert max(feedback['peak_joint_torque_nm']) <= 2.000001
    with pytest.raises(ValueError, match='empty'):
        run_contact(tmp_path/'feedback')


@pytest.mark.parametrize('kwargs,reason', [
    ({'surface_present': False}, 'travel_limit'),
    ({'mode': 'position', 'surface_offset': .006, 'force_limit': 2.7}, 'force_limit'),
    ({'stiffness': 300, 'target_force': 4}, 'probe_stroke_limit'),
])
def test_fault_aborts_and_physically_unloads(tmp_path, kwargs, reason):
    report = run_contact(tmp_path/reason, ContactConfig(scan_seconds=2, **kwargs))
    assert not report['success']
    assert report['controller_abort'] == reason
    assert not report['scan_complete']
    assert report['final_force_n'] < .1
    assert report['unexpected_contact_steps'] == 0


def test_external_stop_latches_and_retracts():
    sim = ContactExperiment(ContactConfig(scan_seconds=2))
    for _ in range(2200):
        sim.step()
    sim.control.abort('external_stop', float(sim.data.time))
    for _ in range(1500):
        sim.step()
    assert sim.control.reason == 'external_stop'
    assert sim.control.phase == 'DONE'
    assert sim.summary()['final_force_n'] < .1


def test_fast_scan_retains_loss_of_contact_failure(tmp_path):
    report = run_contact(tmp_path/'fast', ContactConfig(surface_offset=.002, surface_tilt_deg=0, scan_seconds=2))
    assert not report['success']
    assert 'insufficient_contact' in report['failure_reasons']
    assert report['scan_complete']


def test_initial_penetration_rejected_before_writing_results(tmp_path):
    with pytest.raises(ValueError, match='Initial tool pose'):
        run_contact(tmp_path/'invalid', ContactConfig(surface_offset=.015))
    assert not (tmp_path/'invalid').exists()
