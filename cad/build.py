"""Editable parametric CAD: carrier, contact pad and narrow support fixture.

All dimensions are mm. Run from the project root: python cad/build.py.
Manufacturing STEP and STL plus three-view drawings are generated locally.
"""
from pathlib import Path
import json
import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

ROOT=Path(__file__).resolve().parents[1]
config=json.loads((ROOT/'cad/design.json').read_text(encoding='utf-8'))
OUT=ROOT/'cad/generated'
OUT.mkdir(exist_ok=True)

def build():
    c,p,f=config['carrier'],config['contact_pad'],config['fixture']
    carrier=cq.Workplane('XY').box(c['length'],c['width'],c['thickness'],centered=(True,True,False))
    pad=cq.Workplane('XY').box(p['length'],p['width'],p['thickness'],centered=(True,True,False))
    base=cq.Workplane('XY').box(f['base_length'],f['base_width'],f['base_thickness'],centered=(True,True,False))
    holes=[(sx*f['mount_hole_pitch_x']/2,sy*f['mount_hole_pitch_y']/2) for sx in (-1,1) for sy in (-1,1)]
    base=base.faces('>Z').workplane().pushPoints(holes).hole(f['mount_hole_diameter'])
    pedestal=cq.Workplane('XY',origin=(0,0,f['base_thickness'])).box(f['pedestal_length'],f['pedestal_width'],f['total_height']-f['base_thickness'],centered=(True,True,False))
    fixture=base.union(pedestal)
    assembly=cq.Assembly(name='jaw_contact_attachment')
    assembly.add(carrier,name='carrier_6061',color=cq.Color(0.65,0.72,0.79))
    assembly.add(pad,name='contact_pad_TPU',loc=cq.Location(cq.Vector(0,0,c['thickness'])),color=cq.Color(0.10,0.75,0.65))
    assembly.export(str(OUT/'jaw_attachment.step'))
    parts={'carrier':carrier,'contact_pad':pad,'workpiece_fixture':fixture}
    metrics={}
    for name,part in parts.items():
        assert part.val().isValid(),name
        cq.exporters.export(part,str(OUT/f'{name}.step'))
        cq.exporters.export(part,str(OUT/f'{name}.stl'))
        cq.exporters.export(part,str(OUT/f'{name}_isometric.svg'),exportType='SVG',opt={'width':700,'height':500,'showAxes':False,'projectionDir':(1,-1,0.7)})
        svg=OUT/f'{name}_isometric.svg'
        svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
        metrics[name]={'volume_mm3':part.val().Volume(),'valid_solid':part.val().isValid()}
    metrics['carrier']['mass_g']=metrics['carrier']['volume_mm3']*c['density_kg_m3']/1e6
    metrics['contact_pad']['mass_g']=metrics['contact_pad']['volume_mm3']*p['density_kg_m3']/1e6
    (OUT/'cad_metrics.json').write_text(json.dumps(metrics,indent=2),encoding='utf-8')
    draw(f,c,p)
    print(json.dumps(metrics,indent=2))

def draw(f,c,p):
    plt.rcParams['font.family']='DejaVu Sans'
    fig,axes=plt.subplots(1,3,figsize=(13,5),layout='constrained')
    length,width,height=f['base_length'],f['base_width'],f['total_height']
    pl,pw,bt=f['pedestal_length'],f['pedestal_width'],f['base_thickness']
    ax=axes[0]
    ax.add_patch(Rectangle((-length/2,-width/2),length,width,fill=False,lw=1.5))
    ax.add_patch(Rectangle((-pl/2,-pw/2),pl,pw,fill=False,lw=1.5))
    for x in (-f['mount_hole_pitch_x']/2,f['mount_hole_pitch_x']/2):
        for y in (-f['mount_hole_pitch_y']/2,f['mount_hole_pitch_y']/2):
            ax.add_patch(Circle((x,y),f['mount_hole_diameter']/2,fill=False))
    ax.set_title('Fixture / TOP')
    ax.annotate('',xy=(-length/2,-width/2-5.5),xytext=(length/2,-width/2-5.5),arrowprops={'arrowstyle':'<->'})
    ax.text(0,-width/2-9.5,f'{length:g}',ha='center')
    ax.text(0,width/2+4.5,f"4 x diameter {f['mount_hole_diameter']:g} / pitch {f['mount_hole_pitch_x']:g} x {f['mount_hole_pitch_y']:g}",ha='center',fontsize=9)
    ax.set_xlim(-length/2-10,length/2+10);ax.set_ylim(-width/2-14.5,width/2+12.5)
    ax=axes[1]
    ax.add_patch(Rectangle((-length/2,0),length,bt,fill=False))
    ax.add_patch(Rectangle((-pl/2,bt),pl,height-bt,fill=False))
    ax.annotate('',xy=(length/2+5,0),xytext=(length/2+5,height),arrowprops={'arrowstyle':'<->'})
    ax.text(length/2+7,height/2,f'{height:g}',rotation=90,va='center')
    ax.set_title('Fixture / FRONT')
    task=json.loads((ROOT/'config/task.json').read_text())
    ax.text(0,height+4,f"{pl:g} x {pw:g} support / workpiece {task['workpiece_size'][0]*1000:g} x {task['workpiece_size'][1]*1000:g}",ha='center',fontsize=9)
    ax.set_xlim(-length/2-10,length/2+13);ax.set_ylim(-8,height+11)
    ax=axes[2]
    cl,cw=c['length'],c['width']
    ax.add_patch(Rectangle((-cl/2,-cw/2),cl,cw,facecolor='#35bca6',alpha=.7))
    ax.annotate('',xy=(-cl/2,-cw/2-5),xytext=(cl/2,-cw/2-5),arrowprops={'arrowstyle':'<->'})
    ax.text(0,-cw/2-9,f'{cl:g}',ha='center')
    ax.text(0,cw/2+5,f"{cw:g} wide / carrier {c['thickness']:g} + TPU {p['thickness']:g} thick",ha='center',fontsize=9)
    ax.text(0,cw/2+12,'Bonded interface: hardware fit unverified',ha='center',fontsize=8)
    ax.set_title('Contact attachment / TOP')
    ax.set_xlim(-cl/2-9,cl/2+9);ax.set_ylim(-cw/2-14,cw/2+17)
    for ax in axes:
        ax.set_aspect('equal');ax.set_xlabel('mm');ax.set_ylabel('mm');ax.grid(alpha=.15)
    fig.suptitle('Four-DOF arm - Gripping and positioning parts / Units mm / Rev A',fontsize=15)
    fig.savefig(OUT/'engineering_drawing.png',dpi=180)
    fig.savefig(OUT/'engineering_drawing.pdf')
    plt.close(fig)

if __name__=='__main__':
    build()
