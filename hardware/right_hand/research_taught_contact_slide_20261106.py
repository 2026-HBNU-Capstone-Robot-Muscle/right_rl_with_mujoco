"""Allow contact points to slide as taught fingers curl; offline only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from research_taught_grasp_20261106 import O,R,TEACH,sha,save_json,details
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run
from dynamic_common_grip_search_20261105 import ramp

def main():
    out=O/'sliding_contact';out.mkdir(exist_ok=True);m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));z=np.load(O/'baseline_end.npz');d=restore(m,z);p=restore(m,z)
    hand,_,check=inspect_for(m);cube=m.geom('cube').id;cache=precompute(m,hand);C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();base=d.qpos.copy();pd=d.ctrl-d.qpos[:16];qbase=d.qpos[:16].copy();start=motor(d.ctrl)
    active=np.r_[np.arange(4,8),np.arange(12,16)];taught=ticks_to_sim(np.array(json.loads(TEACH.read_bytes())['corrected_position_ticks']));lo,hi,_,_=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    low=(np.ceil(lo[ids])+8-2048)*TICK-pd;high=(np.floor(hi[ids])-8-2048)*TICK-pd
    movable=[x for x in cache if m.body(int(m.geom_bodyid[x[0]])).name.startswith(('mf_','th_'))];tips=[m.geom(n).id for n in ('mf_tip','th_tip')];cases=[];began=time.perf_counter()
    for fraction,height in ((.5,3.),(1.,3.),(.5,-3.),(1.,-3.)):
        name=f'shape{fraction:g}_height{height:g}';folder=out/name;folder.mkdir(exist_ok=True);prior=qbase.copy();prior[4:8]+=fraction*(taught[4:8]-qbase[4:8]);calls=0
        def measure(x):
            nonlocal calls
            calls+=1;p.qpos[:]=base;p.qpos[active]=x;p.ctrl[:]=p.qpos[:16]+pd;p.qvel[:]=0;mujoco.mj_forward(m,p)
            e=[]
            for g in tips:
                points=np.zeros(6);distance=mujoco.mj_geomDistance(m,p,cube,g,.15,points)*1000
                center=(points[:3]+points[3:])/2;h=-float((center-C)@B[:,1])*1000
                e.extend(((distance+.05)*6,(h-height)*1.,max(0.,h-7.)*10))
            clear=gaps(m,p,C,B,movable,True);su,sp=check(p)
            depth=max((max(0.,-float(c.dist)*1000) for c in p.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return np.r_[e,(x[:4]-prior[4:8])*8,(x[4:]-qbase[12:16])*1.,[max(0.,1.-v)*8 for _,v in clear],max(sp.values(),default=0.)*1500,max(su.values(),default=0.)*1500,max(0.,depth-.6)*1000]
        seeds=[qbase[active],prior[active]];solutions=[]
        for si,seed in enumerate(seeds):
            fit=least_squares(measure,np.clip(seed,low[active]+1e-8,high[active]-1e-8),bounds=(low[active],high[active]),max_nfev=250,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
            solutions.append((float(np.linalg.norm(measure(fit.x))),fit.x.copy(),fit.nfev))
        cost,x,nfev=min(solutions,key=lambda v:v[0]);residual=measure(x);goal=motor(p.ctrl)
        state={k:getattr(p,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};state['time']=p.time;endpoint=details(m,state);clearance=min(v for _,v in endpoint['upper_gaps_mm']);geometry_ok=bool(clearance>.3 and not endpoint['self_contact_mm'] and max(endpoint['support_contact_mm'].values(),default=0.)<=.15 and max(abs(residual[[0,3]]/6))<.8)
        row=dict(name=name,shape_fraction=fraction,target_contact_height_mm=height,static_geometry_only=endpoint,geometry_passed=geometry_ok,cost=cost,optimizer_evaluations=nfev,residual_evaluations=calls,goal=goal.tolist(),free_replays=[])
        for order in ('simultaneous','thumb_first','middle_first'):
            if order=='simultaneous':rows=ramp(start,goal,134)
            else:
                via=start.copy();which=np.arange(4) if order=='thumb_first' else np.arange(8,12);via[which]=goal[which]
                rows=np.vstack([ramp(start,via,20),ramp(via,goal,134)])
            np.savez_compressed(folder/(order+'_path.npz'),start=start,goals=rows)
            dynamic=run(m,z,rows,1,folder/(order+'_end.npz'),folder/(order+'_trajectory.npz'),geometry_ok)
            free=details(m,np.load(folder/(order+'_end.npz')))
            row['free_replays'].append(dict(order=order,dynamic=dynamic,endpoint=free,passed=bool(geometry_ok and dynamic['whole_grip_task_passed'])))
            print(json.dumps(dict(name=name,order=order,geometry_passed=geometry_ok,static_gap=clearance,safety=dynamic['safety_passed'],worst=dynamic['worst'],completed=dynamic['completed_goals'],total=dynamic['total_goals'],tail=dynamic['settled_tail'])),flush=True)
        cases.append(row);save_json(folder/'review.json',row)
        save_json(out/'review.json',dict(status='sliding_lower_contact_research',cases=cases,teaching_as_curvature_reference=True,other_finger_goals_unchanged=True,native_and_contact_criteria_unchanged=True,cube_never_pinned_in_dynamic_replay=True,source_sha256=sha(__import__('pathlib').Path(__file__)),model_sha256=sha(O/'model.mjb'),initial_state_sha256=sha(O/'baseline_end.npz'),teaching_sha256=sha(TEACH),devices_opened=False,hardware_execution_allowed=False,physical_success=False,elapsed_seconds=time.perf_counter()-began))
if __name__=='__main__':main()
