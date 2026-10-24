"""Static trajectory and derivative plots from the saved motion experiments."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from desktop_arm.continuous import build_curve


def trace(output,name):
    d=np.genfromtxt(output/name/'execution/trajectory.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    return d[d['phase']=='TRANSFER']


def plots(output,records):
    colors={'stop':'#af5b36','continuous':'#8e93a2','continuous_ff':'#12789a'}
    families=['spatial_eight','rising_helix','multi_corner']
    fig=plt.figure(figsize=(13,4.5),layout='constrained')
    for i,name in enumerate(families):
        ax=fig.add_subplot(1,3,i+1,projection='3d')
        record=next(r for r in records if r['name']==name+'_continuous_ff')
        ref=build_curve(record['task']).reference(np.linspace(0,1,1200))[:,:3]*1000
        ax.plot(*ref.T,color='#73777c',ls='--',lw=1,label='Reference')
        for mode in ('stop','continuous_ff'):
            d=trace(output,name+'_'+mode)
            xyz=np.array([d['tcp_'+a] for a in 'xyz'])*1000
            ax.plot(*xyz,color=colors[mode],lw=1.2,label=mode)
        ax.set(xlabel='X / mm',ylabel='Y / mm',zlabel='Z / mm',title=name.replace('_',' ').title())
        ax.view_init(elev=25,azim=-60);ax.set_box_aspect([1,1,.7])
    ax.legend(fontsize=8,loc='upper left')
    fig.savefig(output/'spatial_paths.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7),layout='constrained')
    for r in (r for r in records if r['family']=='multi_corner'):
        d=trace(output,r['name']);t=d['time']-d['time'][0];color=colors[r['variant']]
        axes[0,0].plot(t,d['dq2_command'],label=r['variant'],color=color)
        axes[0,1].plot(t,d['jerk2_command'].astype(float),label=r['variant'],color=color)
    axes[0,0].set(xlabel='Transfer time / s',ylabel='Joint 2 command speed / rad/s',title='Stops versus continuous motion')
    axes[0,1].set(xlabel='Transfer time / s',ylabel='Joint 2 command jerk / rad/s3',title='Jerk during the curved transfer')
    axes[0,0].legend(fontsize=8)
    x=np.arange(3)
    for i,mode in enumerate(colors):
        rows=[next(r for r in records if r['name']==n+'_'+mode) for n in families]
        axes[1,0].bar(x+(i-1)*.25,[r['report']['plan']['transfer_duration_s'] for r in rows],width=.24,label=mode,color=colors[mode])
        axes[1,1].bar(x+(i-1)*.25,[r['report']['tracking']['tcp_tracking_rms_mm'] for r in rows],width=.24,label=mode,color=colors[mode])
    for ax in axes[1]:ax.set_xticks(x,['Spatial eight','Rising helix','Multi corner'])
    axes[1,0].set(ylabel='Transfer duration / s',title='Same joint limits; global curve retiming')
    axes[1,1].set(ylabel='TCP tracking RMS / mm',title='Measured versus time-indexed joint command')
    fig.savefig(output/'motion_comparison.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(9,7),layout='constrained')
    d=trace(output,'spatial_eight_continuous_ff');t=d['time']-d['time'][0]
    for i in range(1,5):
        for ax,key in zip(axes,['dq','ddq','jerk']):ax.plot(t,d[f'{key}{i}_command'].astype(float),label=f'Joint {i}')
    for ax,limit,label in zip(axes,[.8,1.5,8],['Speed / rad/s','Acceleration / rad/s2','Jerk / rad/s3']):
        ax.axhline(limit,color='gray',ls='--',lw=.8);ax.axhline(-limit,color='gray',ls='--',lw=.8)
        ax.set(ylabel=label,xlabel='Transfer time / s')
    axes[0].legend(ncol=4,fontsize=9)
    fig.savefig(output/'joint_limits.png',dpi=180);plt.close(fig)
