from collections import Counter
from desktop_arm.simulation import Simulator
from desktop_arm.task import plan_task
import mujoco
sim=Simulator()
pairs=Counter()
for phase in plan_task():
    for i in range(int(phase.segment.duration/sim.model.opt.timestep)+1):
        command,grip=phase.command(i*sim.model.opt.timestep)
        sim.step(command.position,grip)
        for c in sim.data.contact:
            geoms=[int(c.geom1),int(c.geom2)]
            names=[mujoco.mj_id2name(sim.model,mujoco.mjtObj.mjOBJ_GEOM,g) or sim.model.body(sim.model.geom_bodyid[g]).name for g in geoms]
            if any(n in names for n in ('pick_nest','place_nest','floor')):
                pairs[(phase.name,tuple(sorted(names)))]+=1
            if 'workpiece_geom' in names and not any(n in names for n in ('pick_nest','place_nest','floor')):
                pairs[(phase.name,tuple(sorted(names)))]+=1
    print(phase.name,sim.observation()['object'],[(s,float(sim.data.qpos[sim.model.joint(f'gripper_{s}_joint').qposadr[0]])) for s in ('left','right')])
print(pairs)
