"""Add a lower ring support before lowering the index; offline hypotheses."""
from pathlib import Path
import json,time,hashlib,itertools
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from canonical_grip_dynamics_20261105 import run
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from research_probe_index_handoff_20261109 import ramp

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BASE=ROOT/'right_hand_rotation_20261109/index_thumb_transfer/cp1042_together/end.npz'
PARENT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'
INDEX=ROOT/'right_hand_rotation_20261109/lower_index_refinement/height-6/research_path.npz'
OUT=ROOT/'right_hand_rotation_20261109/ring_supported_index_lowering'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(BASE,allow_pickle=False);d=restore(m,z);p=restore(m,z);parent=restore(m,np.load(PARENT,allow_pickle=False))
    parentB=parent.xmat[m.body('cube').id].reshape(3,3).copy();cube=m.geom('cube').id;tip=m.geom('rf_tip').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();qbase=d.qpos[:16].copy();pd=d.ctrl-qbase;active=np.arange(8,12)
    hand,_,check=inspect_for(m);cache=precompute(m,hand);ring=[row for row in cache if m.body(int(m.geom_bodyid[row[0]])).name.startswith('rf_')]
    lo,hi,_,_=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(np.ceil(lo[ids])+4-2048)*TICK-pd;high=(np.floor(hi[ids])-4-2048)*TICK-pd
    start=motor(d.ctrl);index=np.load(INDEX,allow_pickle=False)['goals'];cases=[]
    position=(d.geom_xpos[tip]-C)@B;axis=0 if abs(position[0])>abs(position[2]) else 2;other=2-axis;targets=[]
    for value in (-12.,0.,12.):
        t=np.array([0.,27.5,0.]);t[other]=value;targets.append(('bottom',value,t))
        t=np.zeros(3);t[axis]=np.sign(position[axis])*27.5;t[1]=15.;t[other]=value;targets.append(('side',value,t))
    for role,offset,local in targets:
        target=C+B@local/1000
        def measure(x):
            p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p)
            points=np.zeros(6);distance=mujoco.mj_geomDistance(m,p,cube,tip,.25,points)*1000
            closest=(points[:3]+points[3:])/2;cl=gaps(m,p,C,B,ring,True);su,sp=check(p)
            pen=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[(closest-target)*1000*3,(distance+.05)*6,(x-qbase[active])*.3,
                         [max(0.,.5-v)*5 for _,v in cl],max(sp.values(),default=0.)*2000,max(su.values(),default=0.)*2000,max(0.,pen-.6)*1000]
        seeds=[qbase[active].copy(),qbase[active]+np.array([.5,0,.8,.4])];fits=[]
        for seed in seeds:
            fit=least_squares(measure,np.clip(seed,low[active]+1e-8,high[active]-1e-8),bounds=(low[active],high[active]),max_nfev=120,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
            fitres=measure(fit.x);fits.append((float(np.linalg.norm(fitres)),fit.x.copy(),fit.nfev))
        cost,x,steps=min(fits,key=lambda item:item[0]);res=measure(x);ring_goal=motor(p.ctrl);static_gap=min(value for _,value in gaps(m,p,C,B,ring,True));su,sp=check(p)
        contact_error=float(np.linalg.norm(res[:3])/3);distance_error=float(abs(res[3]/6))
        row={'role':role,'offset_mm':offset,'target_local_mm':local.tolist(),'static_ring_contact_error_mm':contact_error,
             'static_tip_distance_error_mm':distance_error,'static_ring_upper_gap_mm':static_gap,'goal':ring_goal.tolist(),'cost':cost,'optimizer_evaluations':steps}
        folder=OUT/f'{role}{offset:+g}';folder.mkdir(exist_ok=True)
        if contact_error<2 and distance_error<1 and static_gap>.3 and not sp and max(su.values(),default=0.)<=.15:
            ring_rows=np.vstack([ramp(start,ring_goal),np.tile(ring_goal,(27,1))])
            follow=index.copy();follow[:,12:16]=ring_goal[12:16]
            rows=np.vstack([ring_rows,follow]);np.savez_compressed(folder/'research_path.npz',start=start,goals=rows)
            dynamic=run(m,z,rows,1,folder/'end.npz',folder/'trajectory.npz',False,required_lower_contacts=('if_tip','th_tip'))
            end=restore(m,np.load(folder/'end.npz',allow_pickle=False));yaw=angle(end.xmat[m.body('cube').id].reshape(3,3),parentB,z['up'])
            row.update(dynamic=dynamic,cumulative_yaw_from_probe_deg=yaw,coarse_passed=bool(dynamic['whole_grip_task_passed'] and abs(yaw)<5))
        else:row.update(coarse_passed=False,dynamic_not_run_because_geometry_rejected=True)
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'ring_support_before_index_lowering_research','source_model_sha256':sha(SOURCE),'source_state_sha256':sha(BASE),
                'cases':cases,'coarse_passed':sum(case['coarse_passed'] for case in cases),'devices_opened':False,'hardware_execution_allowed':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'role':role,'offset':offset,'contact_error':contact_error,'distance_error':distance_error,'gap':static_gap,
                          'dynamics_run':'dynamic' in row,'passed':row['coarse_passed'],
                          'tilt':row.get('dynamic',{}).get('final_up_tilt_deg')}),flush=True)


if __name__=='__main__':main()
