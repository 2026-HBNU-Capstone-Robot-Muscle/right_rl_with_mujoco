"""Seek a thumb-only lower-body contact while palm/index keep supporting.

Pinned cube IK is only a proposal. Accepted proposals are replayed with a free
cube, active .15Nm model caps and the unchanged native/dynamic guards.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import hashlib,json
import numpy as np,mujoco
from scipy.optimize import least_squares
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parent=ROOT/'near_quarter_full_replay';r=json.loads((parent/'review.json').read_bytes());model=R/r['source_model'];assert sha(model)==r['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/'postturn_middle_clearance/end_state.npz';z=np.load(source);d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];C=base[-7:-4].copy();cube=m.geom('cube').id;B=d.geom_xmat[cube].reshape(3,3).copy();up=z['up'];hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,edges in geometry};gid=m.geom('th_tip').id;lo,hi,nlo,nhi=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;active=np.arange(12,16);low=np.maximum((np.ceil(lo[ids])+8-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-8-2048)*TICK-pd,nhi);out=ROOT/'postturn_thumb_ik';out.mkdir(exist_ok=True);results=[]
    for mode in ('xpos','xneg','zpos','zneg'):
        folder=out/mode;folder.mkdir(exist_ok=True);axis=0 if mode.startswith('x') else 2;normal=B[:,axis]*(1 if mode.endswith('pos') else -1);side=np.cross(up,normal);side/=np.linalg.norm(side)
        def measure(x,exact=False):
            d.qpos[:]=base;d.qpos[active]=x;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);world=vertices[gid]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid];proj=(world-C)@normal;weights=np.exp(-(proj-min(proj))/.0002);point=np.average(world,axis=0,weights=weights);gap=float(min(proj)-.0275)*1000;height=float((point-C)@up)*1000;lateral=float((point-C)@side)*1000;cl=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return gap,height,lateral,point,cl,max(sp.values(),default=0.),max(su.values(),default=0.),pen
        def residual(x):
            gap,height,lateral,point,cl,selfmm,supportmm,pen=measure(x)
            return np.r_[(gap+.05)*10,(height+5)*2,lateral*.2,max(0.,abs(lateral)-20)*10,(x-base[active])*.2,np.maximum(.8-np.asarray([v for name,v in cl]),0)*20,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        fit=least_squares(residual,np.clip(base[active],low[active]+1e-9,high[active]-1e-9),bounds=(low[active],high[active]),max_nfev=180,ftol=1e-9,xtol=1e-9,gtol=1e-9);gap,height,lateral,point,cl,selfmm,supportmm,pen=measure(fit.x,True);accepted=bool(abs(gap)<.2 and -22<height<7 and abs(lateral)<22 and min(v for name,v in cl)>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1);record=dict(mode=mode,static_accepted=accepted,face_gap_mm=gap,contact_height_mm=height,contact_side_mm=lateral,upper_gap_mm=min(v for name,v in cl),self_mm=selfmm,support_mm=supportmm,cube_mm=pen,dynamic=[]);np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B)
        if accepted:
            actual=d.qpos[:16].copy();jac=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jac,None,point,int(m.geom_bodyid[gid]));start=motor(z['ctrl'])
            for force in (.5,1.,1.5):
                control=actual+pd;tau=d.qfrc_bias[active]-d.qfrc_passive[active]-(normal@jac[:,active])*force;control[active]=actual[active]+tau/m.actuator_gainprm[active,0];goal=motor(control)
                if np.any(goal<lo) or np.any(goal>hi):record['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                rows=ramp(start,goal,134);np.savez_compressed(folder/f'force{force:g}.npz',start=start,goals=rows);v=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True);record['dynamic'].append(dict(force_N=force,result=v))
        results.append(record);report=dict(status='thumb_only_lower_contact_IK_proposals_and_free_replays',source_state=str(source.relative_to(R)),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,cube_pose_pinned_only_in_static_IK=True,free_replays_never_reset_cube=True,prior_MF_TH_grip_acceptance_unchanged=True,force_closure_and_actual_left_hand_load_not_certified=True,hardware_execution_allowed=False,devices_opened=False);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(mode=mode,static=accepted,gap=gap,height=height,side=lateral,upper=record['upper_gap_mm'],dynamic=[dict(force_N=t['force_N'],safety=t.get('result',{}).get('safety_passed'),yaw=t.get('result',{}).get('final_yaw_from_placement_deg'),contacts=t.get('result',{}).get('final_contact_N')) for t in record['dynamic']])),flush=True)
if __name__=='__main__':main()
