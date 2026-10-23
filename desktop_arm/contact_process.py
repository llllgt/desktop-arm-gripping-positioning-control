"""Contact-force surface scanning with a passive spring probe on the four-axis arm."""
from collections import deque
from dataclasses import asdict
from pathlib import Path
import csv
import json
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .force_control import ContactConfig, ContactController, ControllerSettings
from .kinematics import inverse
from .paths import project_root
from .scene import build_scene


def load_contact_model(config):
    c = config
    root = ET.fromstring(build_scene(fingers=False))
    root.set('model', 'spring_probe_surface_scan')
    root.find('option').set('timestep', str(c.timestep))
    world = root.find('worldbody')
    for child in list(world):
        if child.get('name', '').startswith(('pick_', 'place_')) or child.get('name') == 'workpiece':
            world.remove(child)
    tip = root.find('.//body[@name="end_effector_link"]')
    # Housing and probe are a conceptual tool mounted to the rigid wrist; they
    # are not held through gripper friction and do not use the transfer pads.
    ET.SubElement(tip, 'geom', name='probe_housing', type='cylinder',
                  pos='0 0 0', quat='0.707106781 0 0.707106781 0',
                  size='0.007 0.008', mass='0.012', rgba='0.55 0.61 0.68 1',
                  contype='1', conaffinity='2')
    slider = ET.SubElement(tip, 'body', name='probe_slider')
    ET.SubElement(slider, 'joint', name='probe_compression', type='slide', axis='-1 0 0',
                  range='-0.002 0.010', stiffness=str(c.stiffness), damping=str(c.damping),
                  springref='0', armature='0', frictionloss='0')
    ET.SubElement(slider, 'geom', name='probe_shaft', type='capsule',
                  fromto='0.004 0 0 0.024 0 0', size='0.002', mass='0.012',
                  rgba='0.7 0.72 0.74 1', contype='1', conaffinity='2')
    ET.SubElement(slider, 'geom', name='probe_ball', type='sphere', pos='0.024 0 0',
                  size='0.004', mass='0.006', rgba='0.95 0.55 0.15 1',
                  contype='1', conaffinity='2', friction='0.2 0.002 0.0001',
                  solref='0.004 1', solimp='0.95 0.99 0.0001')
    ET.SubElement(slider, 'site', name='probe_centre', pos='0.024 0 0', size='0.001')
    if c.surface_present:
        angle = math.radians(c.surface_tilt_deg)
        normal = np.array([math.sin(angle), 0, math.cos(angle)])
        centre = np.array([0.15, 0, c.nominal_surface_z+c.surface_offset])-normal*0.005
        ET.SubElement(world, 'geom', name='process_surface', type='box',
                      pos=' '.join(map(str, centre)), size='0.055 0.025 0.005',
                      quat=f'{math.cos(angle/2)} 0 {math.sin(angle/2)} 0',
                      rgba='0.2 0.5 0.55 1', contype='2', conaffinity='1',
                      friction='0.2 0.002 0.0001', solref='0.004 1',
                      solimp='0.95 0.99 0.0001')
        # A visual stand; the working plate supplies the collision boundary.
        ET.SubElement(world, 'geom', name='surface_stand', type='box',
                      pos='0.15 0 0.023', size='0.045 0.02 0.023',
                      rgba='0.3 0.34 0.38 1', contype='0', conaffinity='0')
    assets = {p.name: p.read_bytes() for p in (project_root()/'assets/upstream/meshes').glob('*.stl')}
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'), assets=assets)


class ForceSensor:
    """Simulated contact-load channel: bias, Gaussian noise, transport delay, LPF."""
    def __init__(self, c):
        self.c = c
        self.rng = np.random.default_rng(c.seed)
        self.delay_steps = round(c.sensor_delay/c.timestep)
        self.queue = deque([0.0]*self.delay_steps)
        self.filtered = 0.0

    def sample(self, actual_force):
        noisy = actual_force + self.c.sensor_bias + self.rng.normal(0, self.c.sensor_noise)
        self.queue.append(float(noisy))
        raw = self.queue.popleft()
        alpha = self.c.timestep/(self.c.filter_tau+self.c.timestep)
        self.filtered += alpha*(raw-self.filtered)
        return raw, self.filtered


def probe_force(model, data):
    """World-frame force on probe from plate, using MuJoCo's geom1->geom2 sign."""
    total, bad = np.zeros(3), set()
    wrench = np.zeros(6)
    for i, contact in enumerate(data.contact):
        names = [model.geom(g).name for g in (contact.geom1, contact.geom2)]
        mujoco.mj_contactForce(model, data, i, wrench)
        if 'process_surface' in names and 'probe_ball' in names:
            sign = 1 if names[1] == 'probe_ball' else -1
            total += sign*(contact.frame.reshape(3, 3).T @ wrench[:3])
        elif 'process_surface' in names and wrench[0] > 1e-5:
            bad.add(tuple(names))
        elif 'floor' in names and wrench[0] > 1e-5:
            other = contact.geom2 if names[0] == 'floor' else contact.geom1
            if model.geom_bodyid[other] != model.body('link1').id:
                bad.add(tuple(names))
    return total, sorted(bad)


class ContactExperiment:
    def __init__(self, config=None):
        self.c = config or ContactConfig()
        self.model = load_contact_model(self.c)
        self.data = mujoco.MjData(self.model)
        self.control = ContactController(ControllerSettings.from_experiment(self.c))
        self.sensor = ForceSensor(self.c)
        self.arm_q = np.array([self.model.joint(f'joint{i}').qposadr[0] for i in range(1, 5)])
        self.arm_d = np.array([self.model.joint(f'joint{i}').dofadr[0] for i in range(1, 5)])
        self.spring_q = self.model.joint('probe_compression').qposadr[0]
        self.command = inverse([self.c.start_x, 0, self.control.start_z], math.pi/2)
        self.data.qpos[self.arm_q] = self.command
        for side in ('left', 'right'):
            self.data.qpos[self.model.joint(f'gripper_{side}_joint').qposadr[0]] = 0.012
        self.data.ctrl[:] = [*self.command, 0.012]
        mujoco.mj_forward(self.model, self.data)
        if self.c.surface_present:
            gap = mujoco.mj_geomDistance(self.model, self.data, self.model.geom('probe_ball').id,
                                        self.model.geom('process_surface').id, 1.0, None)
            if gap < .001 or probe_force(self.model, self.data)[1]:
                raise ValueError('Initial tool pose must clear the surface by at least 1 mm')
        self.bad_steps = 0
        self.peak_force = 0.0
        self.peak_torque = np.zeros(4)
        self.peak_speed = np.zeros(4)
        self.max_compression = 0.0
        self.last_raw = self.last_filtered = 0.0
        self.rows = []

    def step(self):
        force, bad = probe_force(self.model, self.data)
        raw, filtered = self.sensor.sample(force[2])
        now = float(self.data.time)
        self.bad_steps += bool(bad)
        self.peak_force = max(self.peak_force, float(force[2]))
        stroke = float(self.data.qpos[self.spring_q])
        self.max_compression = max(self.max_compression, stroke)
        if bad:
            self.control.abort('unexpected_contact', now)
        if stroke > 0.009:
            self.control.abort('probe_stroke_limit', now)
        xyz = self.control.update(now, filtered, raw)
        try:
            self.command = inverse(xyz, math.pi/2, seed=self.command)
        except ValueError:
            self.control.abort('unreachable_command', now)
        self.data.ctrl[:4] = self.command + self.data.qfrc_bias[self.arm_d]/self.model.actuator_gainprm[:4, 0]
        self.data.ctrl[4] = 0.012
        self.peak_torque = np.maximum(self.peak_torque, np.abs(self.data.qfrc_actuator[self.arm_d]))
        self.peak_speed = np.maximum(self.peak_speed, np.abs(self.data.qvel[self.arm_d]))
        row = dict(time=now, phase=self.control.phase, force_z_true=float(force[2]),
                   force_x_true=float(force[0]), sensor_raw=raw, sensor_filtered=filtered,
                   target_force=self.c.target_force, command_x=float(xyz[0]), command_z=float(xyz[2]),
                   tcp_x=float(self.data.site('tcp').xpos[0]), tcp_z=float(self.data.site('tcp').xpos[2]),
                   compression_m=stroke, unexpected_contacts=len(bad))
        for i in range(4):
            row[f'tau{i+1}'] = float(self.data.qfrc_actuator[self.arm_d[i]])
            row[f'q{i+1}'] = float(self.data.qpos[self.arm_q[i]])
        self.rows.append(row)
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.last_raw, self.last_filtered = raw, filtered
        return row

    def summary(self):
        scan = [r for r in self.rows if r['phase'] == 'SCAN']
        forces = np.array([r['force_z_true'] for r in scan])
        errors = forces-self.c.target_force
        reasons = [] if self.control.reason is None else [self.control.reason]
        if not self.control.scan_complete:
            reasons.append('incomplete_scan')
        rmse = float(np.sqrt(np.mean(errors**2))) if len(scan) else None
        contact_fraction = float(np.mean(forces > 0.1)) if len(scan) else 0.0
        if rmse is None or rmse > 0.3:
            reasons.append('force_tracking_error')
        if contact_fraction < 0.99:
            reasons.append('insufficient_contact')
        if self.bad_steps:
            reasons.append('unexpected_contact')
        final_force = float(probe_force(self.model, self.data)[0][2])
        if final_force > 0.1:
            reasons.append('retraction_incomplete')
        if self.control.phase != 'DONE':
            reasons.append('timeout')
        return dict(engine='MuJoCo '+mujoco.__version__, model_revision='spring-probe-v1',
                    success=not reasons, failure_reasons=sorted(set(reasons)),
                    controller_abort=self.control.reason, scan_complete=self.control.scan_complete,
                    scan_samples=len(scan), scan_force_rmse_n=rmse,
                    scan_mean_force_n=float(np.mean(forces)) if len(scan) else None,
                    scan_contact_fraction=contact_fraction, peak_force_n=self.peak_force,
                    maximum_compression_mm=self.max_compression*1000,
                    peak_joint_torque_nm=self.peak_torque.tolist(), peak_joint_speed_rad_s=self.peak_speed.tolist(),
                    unexpected_contact_steps=self.bad_steps, final_force_n=final_force,
                    simulation_seconds=float(self.data.time), samples=len(self.rows),
                    effective_sensor_delay_s=self.sensor.delay_steps*self.c.timestep,
                    surface_geometry_supplied_to_controller=False,
                    force_channel='simulated probe/plate contact force, world Z')


    def save(self, output):
        output = Path(output)
        report = self.summary()
        with (output/'trace.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        (output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


def run_contact(output, config=None, *, render=False):
    c = config or ContactConfig()
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError('Output directory must be empty')
    experiment = ContactExperiment(c)
    output.mkdir(parents=True, exist_ok=True)
    (output/'config.json').write_text(json.dumps(asdict(c), indent=2), encoding='utf-8')
    renderer = mujoco.Renderer(experiment.model, height=480, width=640) if render else None
    frames = []
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.12, 0, 0.11]
    camera.distance, camera.azimuth, camera.elevation = 0.43, 210, -25
    steps = math.ceil((c.approach_timeout+1.5+c.scan_seconds+4)/c.timestep)
    try:
        for i in range(steps):
            experiment.step()
            if renderer is not None and i % round(0.1/c.timestep) == 0:
                renderer.update_scene(experiment.data, camera=camera)
                frames.append(renderer.render().copy())
            if experiment.control.phase == 'DONE':
                # Allow the physical arm to finish following the retraction command.
                for _ in range(round(0.5/c.timestep)):
                    experiment.step()
                break
    finally:
        if renderer is not None:
            renderer.close()
    report = experiment.summary()
    experiment.save(output)
    if frames:
        import imageio.v2 as imageio
        imageio.mimwrite(output/'demo.gif', frames, duration=100, loop=0)
        imageio.imwrite(output/'preview.png', frames[len(frames)//2])
    return report
