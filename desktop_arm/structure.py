"""Linear elastic tetrahedral FEM and an independent Euler-Bernoulli check.

The carrier is assessed with a deliberately simplified clamped-short-edge
boundary, not a validated model of the bonded attachment or TPU pad.
"""
from pathlib import Path
import json
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve
from .paths import project_root


def beam_mesh(length,width,thickness,nx,ny,nz):
    # x = length, y = load direction / thickness, z = width.
    nodes=np.array([[x,y,z] for x in np.linspace(0,length,nx+1) for y in np.linspace(0,thickness,ny+1) for z in np.linspace(0,width,nz+1)])
    def index(i,j,k):return (i*(ny+1)+j)*(nz+1)+k
    elements=[]
    pattern=[(0,1,3,7),(0,3,2,7),(0,2,6,7),(0,6,4,7),(0,4,5,7),(0,5,1,7)]
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                cube=[index(i,j,k),index(i+1,j,k),index(i,j+1,k),index(i+1,j+1,k),index(i,j,k+1),index(i+1,j,k+1),index(i,j+1,k+1),index(i+1,j+1,k+1)]
                elements.extend([[cube[p] for p in tet] for tet in pattern])
    return nodes,np.array(elements)


def element_matrix(points,E,nu):
    A=np.column_stack([np.ones(4),points])
    volume=abs(np.linalg.det(A))/6
    gradients=np.linalg.inv(A)[1:,:].T
    B=np.zeros((6,12))
    for i,(gx,gy,gz) in enumerate(gradients):
        B[:,3*i:3*i+3]=[[gx,0,0],[0,gy,0],[0,0,gz],[gy,gx,0],[0,gz,gy],[gz,0,gx]]
    lam=E*nu/((1+nu)*(1-2*nu));mu=E/(2*(1+nu))
    D=np.zeros((6,6));D[:3,:3]=lam
    D[np.diag_indices(3)]+=2*mu
    D[3:,3:]=np.eye(3)*mu
    return volume*(B.T@D@B),B,D


def solve_carrier(length=.026,width=.014,thickness=.002,force=6.0,E=69e9,nu=.33,resolution=(26,4,7)):
    nodes,tets=beam_mesh(length,width,thickness,*resolution)
    rows=[];cols=[];values=[]
    for tet in tets:
        K,_,_=element_matrix(nodes[tet],E,nu)
        dofs=(tet[:,None]*3+np.arange(3)).ravel()
        rows.extend(np.repeat(dofs,12));cols.extend(np.tile(dofs,12));values.extend(K.ravel())
    stiffness=coo_matrix((values,(rows,cols)),shape=(len(nodes)*3,)*2).tocsr()
    fixed=np.flatnonzero(np.isclose(nodes[:,0],0))
    tip=np.flatnonzero(np.isclose(nodes[:,0],length))
    fixed_dofs=(fixed[:,None]*3+np.arange(3)).ravel()
    free=np.setdiff1d(np.arange(len(nodes)*3),fixed_dofs)
    loads=np.zeros(len(nodes)*3)
    loads[tip*3+1]=force/len(tip)
    displacement=np.zeros_like(loads)
    displacement[free]=spsolve(stiffness[free][:,free],loads[free])
    reactions=stiffness@displacement-loads
    stress=[];centres=[]
    for tet in tets:
        _,B,D=element_matrix(nodes[tet],E,nu)
        dofs=(tet[:,None]*3+np.arange(3)).ravel()
        sx,sy,sz,txy,tyz,txz=D@B@displacement[dofs]
        stress.append(np.sqrt(.5*((sx-sy)**2+(sy-sz)**2+(sz-sx)**2)+3*(txy*txy+tyz*tyz+txz*txz)))
        centres.append(np.mean(nodes[tet],axis=0))
    inertia=width*thickness**3/12
    analytical=force*length**3/(3*E*inertia)
    result={'resolution':list(resolution),'nodes':len(nodes),'tetrahedra':len(tets),
        'tip_displacement_mm':float(np.mean(displacement[tip*3+1])*1000),
        'beam_theory_tip_mm':analytical*1000,'relative_deflection_difference':float(abs(np.mean(displacement[tip*3+1])/analytical-1)),
        'nominal_beam_root_stress_mpa':6*force*length/(width*thickness**2)/1e6,
        'maximum_von_mises_mpa':float(max(stress)/1e6),
        'reaction_y_n':float(np.sum(reactions[fixed*3+1])),
        'equilibrium_residual_n':float(np.sum(reactions[fixed*3+1])+force)}
    return result,nodes,displacement.reshape(-1,3),np.array(centres),np.array(stress)


def run(output=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=project_root();config=json.loads((root/'cad/design.json').read_text(encoding='utf-8'))
    out=root/'results/structure' if output is None else Path(output)
    out.mkdir(parents=True,exist_ok=True)
    c=config['carrier'];m=config['carrier_material']
    parameters=dict(length=c['length']/1000,width=c['width']/1000,thickness=c['thickness']/1000,force=config['design_load_n'],E=m['young_modulus_pa'],nu=m['poisson_ratio'])
    coarse,_,_,_,_=solve_carrier(**parameters,resolution=(52,8,14))
    fine,nodes,u,centres,stress=solve_carrier(**parameters,resolution=(104,8,14))
    task=json.loads((root/'config/task.json').read_text(encoding='utf-8'))
    required=task['workpiece_mass']*9.81*config['grasp_safety_factor']/(2*config['assumed_contact_friction'])
    report={'boundary':'One short edge fully clamped; 6 N distributed over opposite short edge; small-deformation isotropic 6061-T6 assumption',
        'coarse':coarse,'fine':fine,'mesh_deflection_change':abs(fine['tip_displacement_mm']/coarse['tip_displacement_mm']-1),
        'minimum_per_jaw_normal_force_n':required,'yield_safety_factor_nominal':m['yield_strength_pa']/(fine['nominal_beam_root_stress_mpa']*1e6),
        'scope':'Reference carrier bending calculation only. Adhesive failure, TPU nonlinearity, printed-material anisotropy and actual fit remain unverified.'}
    (out/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    np.savez_compressed(out/'fea_fields.npz',nodes=nodes,displacement=u,element_centres=centres,von_mises=stress)
    fig,ax=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    im=ax[0].scatter(centres[:,0]*1000,centres[:,1]*1000,c=stress/1e6,s=5,cmap='inferno')
    fig.colorbar(im,ax=ax[0],label='von Mises / MPa');ax[0].set_title('Carrier: linear tetrahedral FEM');ax[0].set_xlabel('Length / mm');ax[0].set_ylabel('Thickness / mm')
    unique=np.unique(nodes[:,0])
    measured=[np.mean(u[nodes[:,0]==x,1])*1000 for x in unique]
    L=parameters['length'];I=parameters['width']*parameters['thickness']**3/12
    theory=parameters['force']*unique**2*(3*L-unique)/(6*parameters['E']*I)*1000
    ax[1].plot(unique*1000,theory,label='Euler-Bernoulli theory');ax[1].plot(unique*1000,measured,label='3D tetrahedral FEM');ax[1].set_xlabel('Length / mm');ax[1].set_ylabel('Deflection / mm');ax[1].legend();ax[1].grid(alpha=.2)
    fig.savefig(out/'carrier_analysis.png',dpi=180);plt.close(fig)
    print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    run()
