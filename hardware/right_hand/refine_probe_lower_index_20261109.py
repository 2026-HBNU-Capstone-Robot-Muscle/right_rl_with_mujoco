"""Lower the index contact of the safe IF/TH transfer, with free cube replay."""
from pathlib import Path
import json,time,hashlib
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor,angle
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from research_probe_index_handoff_20261109 import ramp

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe/evidence/model.mjb'
BASE=ROOT/'right_hand_rotation_20261109/index_thumb_transfer/cp1042_together/end.npz'
PARENT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction_linux/checkpoint_1042.npz'
OUT=ROOT/'right_hand_rotation_20261109/lower_index_refinement'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path('source.mjb',assets={'source.mjb':SOURCE.read_bytes()})
    z=np.load(BASE,allow_pickle=False);d=restore(m,z);p=restore(m,z);parent=restore(m,np.load(PARENT,allow_pickle=False))
    C=d.geom_xpos[m.geom('cube').id].copy();B=d.geom_xmat[m.geom('cube').id].reshape(3,3).copy();cube=m.geom('cube').id;tip=m.geom('if_tip').id
    contact=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),key=lambda c:c.dist)
    G=d.geom_xmat[tip].reshape(3,3);n=contact.frame[:3]*(1 if int(contact.geom1)==cube else -1)
    local=G.T@(contact.pos-d.geom_xpos[tip]);localn=G.T@n;clocal=B.T@(contact.pos-C)
    base=d.qpos.copy();qbase=d.qpos[:16].copy();pd=d.ctrl-qbase;active=np.arange(4)
    hand,_,check=inspect_for(m);cache=precompute(m,hand);selected=[row for row in cache if m.body(int(m.geom_bodyid[row[0]])).name.startswith('if_')]
    lo,hi,_,_=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1;low=(np.ceil(lo[ids])+4-2048)*TICK-pd;high=(np.floor(hi[ids])-4-2048)*TICK-pd
    start=motor(d.ctrl);parentB=parent.xmat[m.body('cube').id].reshape(3,3).copy();cases=[]
    for height in (0.,-2.,-4.,-6.):
        targetlocal=clocal.copy();targetlocal[1]=-height/1000;target=C+B@targetlocal
        def measure(x):
            p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p)
            mat=p.geom_xmat[tip].reshape(3,3);point=p.geom_xpos[tip]+mat@local;cl=gaps(m,p,C,B,selected,True);su,sp=check(p)
            pen=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[(point-target)*1000*5,(mat@localn-n)*2,(x-qbase[active])*.5,
                         [max(0.,.6-v)*10 for _,v in cl],max(sp.values(),default=0.)*2000,max(su.values(),default=0.)*2000,max(0.,pen-.6)*1000]
        fit=least_squares(measure,np.clip(qbase[active],low[active]+1e-8,high[active]-1e-8),bounds=(low[active],high[active]),max_nfev=150,ftol=1e-8,xtol=1e-8,gtol=1e-8,diff_step=1e-5)
        residual=measure(fit.x);goal=motor(p.ctrl);cl=sorted(gaps(m,p,C,B,cache,True),key=lambda row:row[1]);err=float(np.linalg.norm(residual[:3])/5);su,sp=check(p)
        geometry=bool(err<.8 and cl[0][1]>.3 and not sp and max(su.values(),default=0.)<=.15)
        folder=OUT/f'height{height:g}';folder.mkdir(exist_ok=True)
        rows=np.vstack([ramp(start,goal),np.tile(goal,(134,1))]);np.savez_compressed(folder/'research_path.npz',start=start,goals=rows)
        dynamic=run(m,z,rows,1,folder/'end.npz',folder/'trajectory.npz',geometry,required_lower_contacts=('if_tip','th_tip'))
        end=restore(m,np.load(folder/'end.npz',allow_pickle=False));combined_yaw=angle(end.xmat[m.body('cube').id].reshape(3,3),parentB,z['up'])
        row={'height_from_cube_center_mm':height,'static_geometry_passed':geometry,'contact_error_mm':err,'static_upper_gaps_mm':cl,
             'goal':goal.tolist(),'dynamic':dynamic,'cumulative_yaw_from_probe_endpoint_deg':combined_yaw,
             'passed':bool(geometry and dynamic['whole_grip_task_passed'] and abs(combined_yaw)<5),
             'only_index_goal_changed':bool(np.array_equal(goal[np.r_[0:4,8:16]],start[np.r_[0:4,8:16]]))}
        cases.append(row);(folder/'review.json').write_text(json.dumps(row,indent=2)+'\n',encoding='utf-8')
        report={'status':'safe_transfer_index_clearance_refinement','source_model_sha256':sha(SOURCE),'source_state_sha256':sha(BASE),
                'cases':cases,'hardware_execution_allowed':False,'devices_opened':False,'elapsed_seconds':time.perf_counter()-began}
        (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({'height':height,'static_ok':geometry,'error_mm':err,'static_gap_mm':cl[0][1],
                          'safety':dynamic['safety_passed'],'combined_yaw_deg':combined_yaw,'passed':row['passed'],'tail':dynamic['settled_tail']}),flush=True)


if __name__=='__main__':main()
