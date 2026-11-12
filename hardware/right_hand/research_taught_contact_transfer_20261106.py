"""Transfer support to index/thumb before clearing the taught middle finger.

All IK is proposal-only. The cube is free in every independently graded replay.
No device APIs or operator command are provided.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_taught_grasp_20261106 import O,R,TEACH,details,sha,save_json
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from dynamic_common_grip_search_20261105 import ramp

def main():
    out=O/'index_contact_transfer';out.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));z=np.load(O/'baseline_end.npz');d=restore(m,z);p=restore(m,z);cube=m.geom('cube').id;hand,_,check=inspect_for(m);cache=precompute(m,hand);C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();pd=d.ctrl-d.qpos[:16];qbase=d.qpos[:16].copy();start=motor(d.ctrl)
    canonical=R/'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/end_state.npz';cz=np.load(canonical);cd=restore(m,cz);CC=cd.geom_xpos[cube].copy();targets=[]
    for name in ('if_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in cd.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);G=cd.geom_xmat[g].reshape(3,3);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);local=G.T@(c.pos-cd.geom_xpos[g]);local_n=G.T@n;cn=B.T@n;axis=max((0,2),key=lambda j:abs(cn[j]));normal=np.zeros(3);normal[axis]=1 if cn[axis]>=0 else -1;cl=B.T@(c.pos-CC);cl=np.clip(cl,-.022,.022);cl[axis]=normal[axis]*.0275;targets.append((g,local,local_n,cl,normal))
    taught=ticks_to_sim(np.array(json.loads(TEACH.read_bytes())['corrected_position_ticks']));active=np.r_[np.arange(8),np.arange(12,16)];lo,hi,_,_=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;lower=(np.ceil(lo[ids])+3-2048)*TICK-pd;upper=(np.floor(hi[ids])-3-2048)*TICK-pd;cases=[];began=time.perf_counter()
    for height in (3.,-3.,-9.):
        folder=out/f'height{height:g}';folder.mkdir(exist_ok=True);prior=qbase.copy();prior[:4]=cd.qpos[:4];prior[12:16]=cd.qpos[12:16];prior[4:8]+=.5*(taught[4:8]-qbase[4:8]);tip_targets=[]
        for g,local,local_n,cl,normal in targets:
            cl=cl.copy();cl[1]=-height/1000;tip_targets.append((g,local,local_n,C+B@cl,B@normal))
        def measure(x):
            p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p);e=[];ne=[]
            for g,local,local_n,target,n in tip_targets:
                G=p.geom_xmat[g].reshape(3,3);e.extend((p.geom_xpos[g]+G@local-target)*1000);ne.extend((G@local_n-n)*2)
            clear=gaps(m,p,C,B,cache,True);su,sp=check(p);depth=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[np.array(e)*5,ne,(x-prior[active])*1.5,[max(0.,1.5-v)*6 for _,v in clear],max(sp.values(),default=0.)*1500,max(su.values(),default=0.)*1500,max(0.,depth-.6)*1000]
        fits=[]
        for seed in (prior[active],qbase[active]):
            fit=least_squares(measure,np.clip(seed,lower[active]+1e-8,upper[active]-1e-8),bounds=(lower[active],upper[active]),max_nfev=250,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5);fits.append((float(np.linalg.norm(measure(fit.x))),fit.x.copy(),fit.nfev))
        cost,x,nfev=min(fits,key=lambda a:a[0]);res=measure(x);goal=motor(p.ctrl);state={k:getattr(p,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};state['time']=p.time;endpoint=details(m,state);error=float(max(np.linalg.norm(res[:6].reshape(2,3)/5,axis=1)));geometry_ok=bool(error<.8 and min(v for _,v in endpoint['upper_gaps_mm'])>.3 and not endpoint['self_contact_mm']);row=dict(contact_height_mm=height,goal=goal.tolist(),geometry_passed=geometry_ok,geometry_error_mm=error,geometry_only=endpoint,canonical_contact_reference_sha256=sha(canonical),cases=[])
        for order in ('simultaneous','index_thumb_then_middle','index_then_thumb_then_middle'):
            if order=='simultaneous':rows=ramp(start,goal,134)
            else:
                via=start.copy();via[4:8]=goal[4:8]
                if order=='index_thumb_then_middle':via[:4]=goal[:4];rows=np.vstack([ramp(start,via,20),ramp(via,goal,134)])
                else:
                    via2=via.copy();via2[:4]=goal[:4];rows=np.vstack([ramp(start,via,20),ramp(via,via2,20),ramp(via2,goal,134)])
            name=order;np.savez_compressed(folder/(name+'_path.npz'),start=start,goals=rows);result=run(m,z,rows,1,folder/(name+'_end.npz'),folder/(name+'_trajectory.npz'),geometry_ok,5,('if_tip','th_tip'));row['cases'].append(dict(order=order,dynamic=result,endpoint=details(m,np.load(folder/(name+'_end.npz')))));print(json.dumps(dict(height=height,order=order,geometry_ok=geometry_ok,geometry_error_mm=error,safety=result['safety_passed'],completed=result['completed_goals'],total=result['total_goals'],task=result['whole_grip_task_passed'],contacts=result['final_contact_N'],tail=result['settled_tail'])),flush=True)
        cases.append(row);save_json(folder/'review.json',row);save_json(out/'review.json',dict(status='index_lower_support_then_middle_clearance_research',cases=cases,source_model_sha256=sha(O/'model.mjb'),source_state_sha256=sha(O/'baseline_end.npz'),source_script_sha256=sha(Path(__file__)),native_and_dynamic_guard_criteria_unchanged=True,required_lower_roles=['if_tip','th_tip'],roles_changed_explicitly_from_middle_thumb=True,devices_opened=False,hardware_execution_allowed=False,physical_success=False,elapsed_seconds=time.perf_counter()-began))
if __name__=='__main__':main()
