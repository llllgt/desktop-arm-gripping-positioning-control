"""Keep the ROS description consistent with the contact/CAD attachment."""
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
design=json.loads((ROOT/'cad/design.json').read_text(encoding='utf-8'))
root=ET.parse(ROOT/'assets/upstream/robot.urdf').getroot()
root.set('name','desktop_arm')
for mesh in root.findall('.//mesh'):
    mesh.set('filename',mesh.get('filename').replace('package://open_manipulator_description/meshes/open_manipulator_x','package://desktop_arm/assets/upstream/meshes'))
for joint in root.findall('joint[@type="revolute"]'):
    joint.find('limit').set('effort','2.0')
    joint.find('limit').set('velocity','0.8')
for side,sign in [('left',1),('right',-1)]:
    name=f'{side}_contact_attachment'
    link=ET.SubElement(root,'link',name=name)
    c,p=design['carrier'],design['contact_pad']
    for part,color,y in [(c,'0.65 0.72 0.79 1',sign*(.0031-c['thickness']/2000)),(p,'0.1 0.75 0.65 1',sign*(.0031-c['thickness']/1000-p['thickness']/2000))]:
        size=f"{part['length']/1000} {part['thickness']/1000} {part['width']/1000}"
        for tag in ('visual','collision'):
            elem=ET.SubElement(link,tag)
            ET.SubElement(elem,'origin',xyz=f'0.050 {y} 0',rpy='0 0 0')
            ET.SubElement(ET.SubElement(elem,'geometry'),'box',size=size)
            if tag=='visual':
                ET.SubElement(ET.SubElement(elem,'material',name=name+str(y)),'color',rgba=color)
    parts=[]
    for part,y in [(c,sign*(.0031-c['thickness']/2000)),(p,sign*(.0031-c['thickness']/1000-p['thickness']/2000))]:
        dims=np.array([part['length'],part['thickness'],part['width']])/1000
        mass=float(np.prod(dims)*part['density_kg_m3'])
        parts.append((mass,np.array([.05,y,0]),dims))
    mass=sum(v[0] for v in parts)
    centre=sum(m*pos for m,pos,_ in parts)/mass
    inertia=np.zeros((3,3))
    for m,pos,dims in parts:
        x,y,z=dims;offset=pos-centre
        inertia+=np.diag(m/12*np.array([y*y+z*z,x*x+z*z,x*x+y*y]))
        inertia+=m*(np.dot(offset,offset)*np.eye(3)-np.outer(offset,offset))
    inertial=ET.SubElement(link,'inertial')
    ET.SubElement(inertial,'origin',xyz=' '.join(map(str,centre)))
    ET.SubElement(inertial,'mass',value=str(mass))
    ET.SubElement(inertial,'inertia',**{key:str(inertia[i,j]) for key,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]})
    joint=ET.SubElement(root,'joint',name=name+'_fixed',type='fixed')
    ET.SubElement(joint,'parent',link=f'gripper_{side}_link');ET.SubElement(joint,'child',link=name)
ET.indent(root)
ET.ElementTree(root).write(ROOT/'assets/desktop_arm.urdf',encoding='utf-8',xml_declaration=True)
print('URDF generated')
