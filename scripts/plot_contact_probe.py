"""Draw the equivalent probe model and its quasistatic spring travel tradeoff."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np
from desktop_arm.contact_process import load_contact_model
from desktop_arm.force_control import ContactConfig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    m=load_contact_model(ContactConfig())
    housing=m.geom('probe_housing');ball=m.geom('probe_ball')
    radius,half_length=housing.size[:2]*1000
    centre=ball.pos[0]*1000;ball_r=ball.size[0]*1000
    fig,axes=plt.subplots(1,2,figsize=(10,4.2),layout='constrained')
    ax=axes[0]
    ax.add_patch(Rectangle((-half_length,-radius),2*half_length,2*radius,fill=False,lw=2,color='#55636e'))
    ax.plot([4,centre],[0,0],lw=5,color='#777777')
    ax.add_patch(Circle((centre,0),ball_r,color='#df8d36'))
    ax.annotate('TCP / housing',xy=(0,radius),xytext=(-7,18),arrowprops={'arrowstyle':'-'})
    ax.annotate('Ball diameter 8 mm',xy=(centre,ball_r),xytext=(10,24),arrowprops={'arrowstyle':'-'})
    ax.annotate('',xy=(0,-12),xytext=(centre+ball_r,-12),arrowprops={'arrowstyle':'<->'})
    ax.text((centre+ball_r)/2,-17,'TCP to tip: 28 mm',ha='center')
    ax.annotate('',xy=(centre-10,11),xytext=(centre,11),arrowprops={'arrowstyle':'->'})
    ax.text(centre-5,14,'Compression',ha='center',fontsize=9)
    ax.set(xlim=(-15,42),ylim=(-22,30),aspect='equal',title='Equivalent axial tool geometry',xlabel='Local tool x / mm',ylabel='Radial distance / mm')
    ax.text(-12,-21,'Housing mass 12 g; moving probe mass 18 g',fontsize=8)
    ax=axes[1]
    x=np.linspace(0,10,101)
    for k in (400,800,1600):
        ax.plot(x,k*x/1000+.018*9.81,label=f'{k} N/m')
    ax.axhline(2,color='black',ls='--',lw=1,label='2 N target')
    ax.axvline(9,color='gray',ls=':',label='9 mm stop threshold')
    ax.set(xlabel='Compression / mm',ylabel='Vertical contact force / N',title='Static estimate: spring load + probe weight',ylim=(0,8))
    ax.legend(fontsize=9)
    fig.savefig(args.output,dpi=180)
    plt.close(fig)


if __name__=='__main__':
    main()
