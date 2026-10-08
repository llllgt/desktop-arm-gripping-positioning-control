"""Build a real contact dynamics scene; no object welding or pose teleportation."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .paths import project_root
from .task import load_task, validate_task


def build_scene(task=None, *, fingers=True, output=None):
    rootpath = project_root()
    task = load_task() if task is None else validate_task(task)
    design = __import__('json').loads((rootpath/'cad/design.json').read_text(encoding='utf-8'))
    fixture = design['fixture']
    root = ET.parse(rootpath / 'assets/upstream/robot.xml').getroot()
    root.set('model', 'desktop_workpiece_transfer')
    root.find('compiler').set('meshdir', '')
    for mesh in root.findall('./asset/mesh'):
        mesh.set('file', Path(mesh.get('file')).name)
    option = root.find('option')
    option.set('timestep', '0.002')
    option.set('integrator', 'implicitfast')
    option.set('iterations', '80')
    # Collision category 1=robot, 2=workpiece/fixtures. Keep self collision active.
    for geom in root.findall('.//geom'):
        if geom.get('group') == '3':
            geom.set('contype', '1')
            geom.set('conaffinity', '3')
            geom.set('friction', '0.9 0.01 0.001')
    # Each upstream jaw mesh contains linkage and an open cavity. A single
    # convex hull fills that cavity and can grasp through empty space. Use an
    # explicit distal metal contact plate, with dimensions extracted from STL,
    # while retaining the original mesh for visualization.
    for side, sign in [('left',1),('right',-1)]:
        jaw = root.find(f'.//body[@name="gripper_{side}_link"]')
        jaw.find('geom[@group="3"]').set('contype','0')
        jaw.find('geom[@group="3"]').set('conaffinity','0')
        ET.SubElement(jaw,'geom',name=f'{side}_metal_contact',type='box',pos=f'0.0469 {sign*0.0041} 0',size='0.0075 0.001 0.029',rgba='0.4 0.4 0.4 0',contype='1',conaffinity='2',friction='0.6 0.01 0.001',group='3',density='0')
    for actuator in root.findall('./actuator/position'):
        arm = actuator.get('joint').startswith('joint')
        actuator.set('kp', '220' if arm else '800')
        actuator.set('forcerange', '-2 2' if arm else '-6 6')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0.1 -0.2 1.2', dir='0 0 -1', directional='true', diffuse='0.9 0.9 0.9')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='0.6 0.6 0.01', rgba='0.09 0.13 0.19 1', contype='2', conaffinity='1')
    for label in ('pick', 'place'):
        p = task[label]
        if not np.isclose(p[2]-task['workpiece_size'][2]/2,fixture['total_height']/1000,atol=1e-9):
            raise ValueError('Task workpiece centre must agree with the CAD support height')
        halfheight = (p[2] - task['workpiece_size'][2]/2)/2
        ET.SubElement(world, 'geom', name=label+'_nest', type='box', pos=f'{p[0]} {p[1]} {halfheight}', size=f"{fixture['pedestal_length']/2000} {fixture['pedestal_width']/2000} {halfheight}", rgba='0.22 0.32 0.42 1', contype='2', conaffinity='3', friction='0.9 0.01 0.001')
        halfbase=fixture['base_thickness']/2000
        ET.SubElement(world, 'geom', name=label+'_baseplate', type='box', pos=f'{p[0]} {p[1]} {halfbase}', size=f"{fixture['base_length']/2000} {fixture['base_width']/2000} {halfbase}", rgba='0.22 0.32 0.42 1', contype='2', conaffinity='3')
        ET.SubElement(world, 'site', name=label+'_target', pos=' '.join(map(str,p)), size='0.002', rgba='0.2 0.7 1 1')
    object_position = np.array(task['pick']) + np.array(task.get('initial_workpiece_offset',[0,0,0]))
    obj = ET.SubElement(world, 'body', name='workpiece', pos=' '.join(map(str,object_position)))
    ET.SubElement(obj, 'freejoint', name='workpiece_free')
    ET.SubElement(obj, 'geom', name='workpiece_geom', type='box', size=' '.join(str(s/2) for s in task['workpiece_size']), mass=str(task['workpiece_mass']), rgba='0.95 0.48 0.12 1', contype='2', conaffinity='3', friction='1.2 0.02 0.002', condim='4', solref='0.008 1')
    tip = root.find('.//body[@name="end_effector_link"]')
    for geom in list(tip):
        tip.remove(geom)
    ET.SubElement(tip, 'site', name='tcp', size='0.003', rgba='0.1 0.85 0.7 1')
    if fingers:
        # Add thin replaceable pads at the jaws' distal contact surfaces.
        # All dimensions and coordinates mirror the parametric CAD source.
        for side, sign in [('left', 1), ('right', -1)]:
            jaw = root.find(f'.//body[@name="gripper_{side}_link"]')
            attachment = ET.SubElement(jaw,'body',name=f'{side}_attachment')
            c,p=design['carrier'],design['contact_pad']
            thickness=c['thickness']/1000
            carrier_y=sign*(.0031-thickness/2)
            pad_y=sign*(.0031-thickness-p['thickness']/2000)
            carrier_mass=c['length']*c['width']*c['thickness']*c['density_kg_m3']/1e9
            pad_mass=p['length']*p['width']*p['thickness']*p['density_kg_m3']/1e9
            ET.SubElement(attachment,'geom',name=f'{side}_carrier',type='box',pos=f'0.050 {carrier_y} 0',size=f"{c['length']/2000} {c['thickness']/2000} {c['width']/2000}",rgba='0.65 0.72 0.79 1',contype='0',conaffinity='0',mass=str(carrier_mass))
            ET.SubElement(attachment, 'geom', name=f'{side}_pad', type='box', pos=f'0.050 {pad_y} 0', size=f"{p['length']/2000} {p['thickness']/2000} {p['width']/2000}", rgba='0.1 0.75 0.65 1', contype='1', conaffinity='2', friction=f"{task.get('contact_friction',design['assumed_contact_friction'])} 0.02 0.002", condim='4', mass=str(pad_mass))
        # Equal-priority dynamic geom contacts take max friction. Explicit pairs
        # let the task set pad/object sliding friction even below object friction.
        # Keep the old mixed solver parameters (0.02 and 0.008 -> 0.014).
        contact = root.find('contact')
        if contact is None:
            contact = ET.SubElement(root, 'contact')
        mu = task.get('contact_friction', design['assumed_contact_friction'])
        for side in ('left', 'right'):
            ET.SubElement(contact, 'pair', name=f'{side}_grasp', geom1=f'{side}_pad',
                          geom2='workpiece_geom', condim='4',
                          friction=f'{mu} {mu} 0.02 0.002 0.002', solref='0.014 1',
                          solimp='0.9 0.95 0.001 0.5 2', margin='0', gap='0')
    ET.SubElement(root, 'statistic', center='0.12 0 0.12', extent='0.5')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720', azimuth='135', elevation='-25')
    ET.SubElement(visual, 'headlight', ambient='0.4 0.4 0.4', diffuse='0.7 0.7 0.7')
    ET.SubElement(root.find('asset'),'texture',name='sky',type='skybox',builtin='gradient',rgb1='0.22 0.29 0.39',rgb2='0.07 0.10 0.16',width='512',height='3072')
    # Scene object adds 7 free-joint qpos; the upstream keyframe no longer applies.
    keyframe = root.find('keyframe')
    root.remove(keyframe)
    xml = ET.tostring(root, encoding='unicode')
    if output is not None:
        Path(output).write_text(xml, encoding='utf-8')
    return xml


def load_model(task=None, fingers=True):
    import mujoco
    # Feed bytes through MuJoCo's virtual filesystem: native fopen on Windows
    # does not reliably support the Chinese characters in this workspace path.
    assets = {p.name:p.read_bytes() for p in (project_root()/'assets/upstream/meshes').glob('*.stl')}
    return mujoco.MjModel.from_xml_string(build_scene(task,fingers=fingers),assets=assets)
