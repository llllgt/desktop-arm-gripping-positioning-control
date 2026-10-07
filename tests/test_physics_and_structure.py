import json
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from desktop_arm.simulation import run
from desktop_arm.structure import element_matrix, solve_carrier
from desktop_arm.scene import load_model
from desktop_arm.paths import project_root


def test_real_contact_transfer_with_actuator_limits(tmp_path):
    result=run(tmp_path)
    assert result['success']
    assert result['object_attachment_used'] is False
    assert result['pad_contact_samples']>100
    assert result['unwanted_contact_samples']==0
    assert result['placement_error_mm']<3
    data=np.genfromtxt(tmp_path/'trajectory.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    for i in range(1,5):
        assert np.max(np.abs(data[f'tau{i}']))<=2.000001


def test_cad_dimensions_and_mass_are_used_in_contact_model():
    design=json.loads((project_root()/'cad/design.json').read_text())
    m=load_model()
    for side in ('left','right'):
        np.testing.assert_allclose(m.geom(f'{side}_pad').size,[design['contact_pad']['length']/2000,design['contact_pad']['thickness']/2000,design['contact_pad']['width']/2000])
        assert abs(m.body(f'{side}_attachment').mass-.0028028)<1e-9


def test_exported_urdf_attachment_mass_and_inertia_match_contact_model():
    root=ET.parse(project_root()/'assets/desktop_arm.urdf').getroot()
    model=load_model()
    for side in ('left','right'):
        inertial=root.find(f"link[@name='{side}_contact_attachment']/inertial")
        body=model.body(f'{side}_attachment')
        assert abs(float(inertial.find('mass').get('value'))-body.mass)<1e-12
        np.testing.assert_allclose(np.fromstring(inertial.find('origin').get('xyz'),sep=' '),body.ipos,atol=1e-12)
        inertia=inertial.find('inertia')
        components={name:float(inertia.get(name)) for name in ('ixx','iyy','izz','ixy','ixz','iyz')}
        exported=np.array([[components['i'+''.join(sorted(a+b))] for b in 'xyz'] for a in 'xyz'])
        np.testing.assert_allclose(np.sort(np.linalg.eigvalsh(exported)),np.sort(body.inertia),atol=1e-12)


def test_tetrahedral_element_rigid_modes_and_uniform_strain():
    pts=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],dtype=float)
    K,B,D=element_matrix(pts,69e9,.33)
    np.testing.assert_allclose(K,K.T,rtol=1e-12,atol=1e-4)
    eig=np.linalg.eigvalsh(K)
    assert np.sum(np.abs(eig)<1e-4)==6
    translation=np.tile([.1,.2,.3],4)
    np.testing.assert_allclose(B@translation,0,atol=1e-12)
    # Uniform x extension and zero other strains is a physical patch test.
    displacement=np.column_stack([pts[:,0]*.001,np.zeros(4),np.zeros(4)]).ravel()
    np.testing.assert_allclose(B@displacement,[.001,0,0,0,0,0],atol=1e-12)


def test_fem_force_balance_and_load_scaling():
    a=solve_carrier(force=3,resolution=(8,2,3))[0]
    b=solve_carrier(force=6,resolution=(8,2,3))[0]
    assert abs(b['equilibrium_residual_n'])<1e-6
    assert abs(b['tip_displacement_mm']/a['tip_displacement_mm']-2)<1e-7
