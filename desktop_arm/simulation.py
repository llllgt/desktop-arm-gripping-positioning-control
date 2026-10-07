"""Native MuJoCo executor, including contacts, actuator limits and measured logs."""
from pathlib import Path
import csv
import json
import time
import numpy as np
import mujoco
from .scene import load_model
from .task import load_task, plan_task
from .kinematics import forward


class Simulator:
    def __init__(self, task=None, fingers=True):
        self.task = load_task() if task is None else task
        self.model = load_model(self.task, fingers=fingers)
        self.data = mujoco.MjData(self.model)
        self.arm_qpos = np.array([self.model.joint(f'joint{i}').qposadr[0] for i in range(1,5)])
        self.arm_dof = np.array([self.model.joint(f'joint{i}').dofadr[0] for i in range(1,5)])
        self.object_body = self.model.body('workpiece').id
        self.tcp_site = self.model.site('tcp').id
        self._reset()

    def _reset(self):
        mujoco.mj_resetData(self.model, self.data)
        home = plan_task(self.task)[0].segment.start
        self.data.qpos[self.arm_qpos] = home
        for side in ('left', 'right'):
            self.data.qpos[self.model.joint(f'gripper_{side}_joint').qposadr[0]] = self.task['gripper_open']
        self.data.ctrl[:] = [*home, self.task['gripper_open']]
        mujoco.mj_forward(self.model, self.data)

    def step(self, command, gripper, gravity_compensation=True):
        self.data.ctrl[:4] = command
        self.data.ctrl[4] = gripper
        self.data.qfrc_applied[:] = 0
        if gravity_compensation:
            # Feedforward is applied through the position actuator target, so its
            # force/torque limits still constrain the COMPLETE control effort.
            gains = self.model.actuator_gainprm[:4, 0]
            self.data.ctrl[:4] += self.data.qfrc_bias[self.arm_dof] / gains
        mujoco.mj_step(self.model, self.data)

    def observation(self):
        return {'q': self.data.qpos[self.arm_qpos].copy(),
                'dq': self.data.qvel[self.arm_dof].copy(),
                'tcp': self.data.site_xpos[self.tcp_site].copy(),
                'object': self.data.xpos[self.object_body].copy()}


def run(output, *, task=None, order=5, render=False, fingers=True, gravity_compensation=True):
    task = load_task() if task is None else task
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    sim = Simulator(task, fingers)
    phases = plan_task(task, order)
    rows, frames = [], []
    writer = None
    preview = None
    if render:
        import imageio.v2 as imageio
        writer = imageio.get_writer(output/'demo.mp4',fps=1/(round(1/(30*sim.model.opt.timestep))*sim.model.opt.timestep),codec='libx264',quality=8)
    renderer = mujoco.Renderer(sim.model, height=720, width=1280) if render else None
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.13, 0.01, 0.12]
    camera.distance, camera.azimuth, camera.elevation = 0.68, 135, -25
    started = time.perf_counter()
    frame_stride = round(1/(30*sim.model.opt.timestep))
    step_count = 0
    try:
        for phase in phases:
            n = int(np.ceil(phase.segment.duration / sim.model.opt.timestep))
            for i in range(n + 1):
                command, opening = phase.command(min(i*sim.model.opt.timestep, phase.segment.duration))
                sim.step(command.position, opening, gravity_compensation)
                obs = sim.observation()
                contacts = 0
                bad_contacts = 0
                for c in sim.data.contact:
                    names = {mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (c.geom1,c.geom2)}
                    if 'workpiece_geom' in names and any(n in names for n in ('left_pad','right_pad')):
                        contacts += 1
                    if not ('workpiece_geom' in names) and any(n in names for n in ('pick_nest','place_nest','pick_baseplate','place_baseplate','floor')):
                        # Robot base/floor contact is intentional; any other link hitting fixtures isn't.
                        for geom_id in (c.geom1, c.geom2):
                            body = sim.model.geom_bodyid[int(geom_id)]
                            if body not in (0, sim.model.body('link1').id):
                                bad_contacts += 1
                if step_count % 5 == 0:
                    row = {'time':float(sim.data.time), 'phase':phase.name, 'gripper_command':opening, 'pad_contacts':contacts, 'unwanted_contacts':bad_contacts}
                    for j in range(4):
                        row.update({f'q{j+1}_command':command.position[j], f'q{j+1}':obs['q'][j], f'dq{j+1}':obs['dq'][j], f'tau{j+1}':sim.data.qfrc_actuator[sim.arm_dof[j]] + sim.data.qfrc_applied[sim.arm_dof[j]]})
                    for k, axis in enumerate('xyz'):
                        row['tcp_'+axis] = obs['tcp'][k]
                        row['target_'+axis] = forward(command.position).position[k]
                        row['object_'+axis] = obs['object'][k]
                    rows.append(row)
                if renderer is not None and step_count % frame_stride == 0:
                    renderer.update_scene(sim.data, camera=camera)
                    frame = renderer.render().copy()
                    writer.append_data(frame)
                    if step_count % (frame_stride*3) == 0:
                        frames.append(frame[::2,::2])
                    if preview is None and phase.name == 'TRANSFER':
                        preview = frame
                step_count += 1
    finally:
        if renderer is not None:
            renderer.close()
        if writer is not None:
            writer.close()
    with (output/'trajectory.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    actual = np.array([[r[f'q{i}'] for i in range(1,5)] for r in rows])
    target = np.array([[r[f'q{i}_command'] for i in range(1,5)] for r in rows])
    errors = np.array([[r['tcp_'+a]-r['target_'+a] for a in 'xyz'] for r in rows])
    final = sim.observation()['object']
    object_track = np.array([[r['object_'+a] for a in 'xyz'] for r in rows])
    distance = float(np.linalg.norm(final - task['place']))
    lifted = float(np.max(object_track[:,2]) - task['pick'][2])
    summary = {'engine':'MuJoCo '+mujoco.__version__, 'order':order, 'fingers':fingers,
        'gravity_compensation':gravity_compensation, 'simulation_seconds':float(sim.data.time),
        'wall_seconds':time.perf_counter()-started, 'samples':len(rows),
        'joint_rmse_rad':np.sqrt(np.mean((actual-target)**2,axis=0)).tolist(),
        'tcp_rmse_mm':float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))*1000),
        'final_workpiece_xyz_m':final.tolist(), 'placement_error_mm':distance*1000,
        'maximum_lift_mm':lifted*1000, 'pad_contact_samples':sum(r['pad_contacts']>0 for r in rows),
        'unwanted_contact_samples':sum(r['unwanted_contacts']>0 for r in rows),
        'success':bool(distance < 0.008 and lifted > task['clearance']*0.6 and all(r['unwanted_contacts']==0 for r in rows)),
        'object_attachment_used':False}
    (output/'summary.json').write_text(json.dumps(summary, indent=2),encoding='utf-8')
    if frames:
        import imageio.v2 as imageio
        imageio.imwrite(output/'preview.png',preview if preview is not None else frames[len(frames)//2])
        imageio.mimwrite(output/'demo.gif',frames,duration=frame_stride*sim.model.opt.timestep*3,loop=0)
    return summary
