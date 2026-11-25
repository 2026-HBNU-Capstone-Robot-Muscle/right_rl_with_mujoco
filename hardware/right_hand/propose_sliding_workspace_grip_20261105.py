"""Seek lower-body face contacts with free sliding points and joint headroom.

Static proposal only. All collision geometry and bounds remain unchanged, and
no accepted endpoint stands in for continuous free-cube motion.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--weight',type=float,choices=(1.,3.,10.),required=True);ap.add_argument('--seed',choices=('current','open','raised'),default='open');ap.add_argument('--mode',choices=('source','xpos','xneg','zpos','zneg'),default='source');ap.add_argument('--finger',choices=('middle','index'),default='middle');a=ap.parse_args();out=ROOT/f'sliding_workspace_grip/weight{a.weight:g}_{a.seed}{"_"+a.mode if a.mode!="source" else ""}{"_index" if a.finger=="index" else ""}';out.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';m=mujoco.MjModel.from_binary_path(str(folder/'model.mjb'));z=np.load(folder/'end_state.npz');d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C0=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();quat=Rotation.from_matrix(B).as_quat();up=-B[:,1];lo,hi,nlo,nhi=native(m);ids=np.array(SIM_TO_MOTOR_IDS)-1
    if a.finger=='index':
        # Free the middle tip from its loaded equilibrium before evaluating a
        # different endpoint. This remains an unvalidated transition proposal.
        base[4:8]=d.ctrl[4:8];base[6]-=96*TICK;pd[4:8]=0.
    low=np.maximum((np.ceil(lo[ids])+4-2048)*TICK-pd,nlo);high=np.minimum((np.floor(hi[ids])-4-2048)*TICK-pd,nhi);finger_active=np.arange(4,8) if a.finger=='middle' else np.arange(4);A=np.r_[0,np.arange(4,8),np.arange(12,16)] if a.finger=='middle' else np.r_[finger_active,np.arange(12,16)];K=len(A);hand,_,check=inspect_for(m);geometry=precompute(m,hand);vertices={g:v for g,v,edges in geometry};contacts=[]
    for role,name in enumerate(('mf_tip' if a.finger=='middle' else 'if_tip','th_tip')):
        gid=m.geom(name).id
        if a.mode=='source':
            c=min((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,gid}),key=lambda c:c.dist);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);local=B.T@n;which=int(np.argmax(abs(local)));normal=B[:,which]*np.sign(local[which])
        else:
            which=0 if a.mode.startswith('x') else 2;sign=1 if a.mode.endswith('pos') else -1;normal=B[:,which]*sign*(1 if role==0 else -1)
        horizontal=np.cross(up,normal);horizontal/=np.linalg.norm(horizontal);contacts.append((gid,normal,horizontal))
    desired=motor(d.ctrl)
    if a.finger=='middle':desired[9]=2300;desired[10]=2700;desired[8]=2900;desired[11]=2550
    else:desired[7]=2500;desired[5]=2200;desired[6]=2700;desired[4]=2700
    preferred=(desired[ids]-2048)*TICK-pd
    seed=np.r_[base[A] if a.seed=='current' else np.clip(preferred[A],low[A]+1e-8,high[A]-1e-8),up*(10 if a.seed=='raised' else 0)];lower=np.r_[low[A],[-45.]*3];upper=np.r_[high[A],[45.]*3]
    def measure(x,exact=False):
        C=C0+x[-3:]/1000;d.qpos[:]=base;d.qpos[A]=x[:K];d.qpos[-7:]=np.r_[C,quat[3],quat[:3]];d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);tasks=[];details=[]
        for gid,normal,horizontal in contacts:
            world=vertices[gid]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid];projection=(world-C)@normal;distance=float(min(projection)-.0275);weights=np.exp(-(projection-min(projection))/.0002);point=np.average(world,axis=0,weights=weights);height=float((point-C)@up)*1000;side=float((point-C)@horizontal)*1000;tasks.extend([(distance*1000+.05)*5,side*.4,(height+2)*.5]);details.append(dict(geom=m.geom(gid).name,face_gap_mm=distance*1000,point_height_mm=height,point_lateral_mm=side))
        clear=gaps(m,d,C,B,geometry,exact);su,sp=check(d);pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
        return np.array(tasks),details,clear,max(sp.values(),default=0.),max(su.values(),default=0.),pen
    def residual(x):
        tasks,details,clear,selfmm,supportmm,pen=measure(x);joint=(x[:K]-preferred[A])*a.weight;inside=[max(0,abs(c['point_lateral_mm'])-22)*10 for c in details];coupled=[] if a.mode=='source' else [(details[0]['point_lateral_mm']+details[1]['point_lateral_mm'])*1.5,(details[0]['point_height_mm']-details[1]['point_height_mm'])*.5];return np.r_[tasks,joint,inside,coupled,np.maximum(.8-np.array([v for name,v in clear]),0)*10,x[-3:]*.01,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
    began=time.perf_counter();fit=least_squares(residual,seed,bounds=(lower,upper),max_nfev=220,ftol=1e-8,xtol=1e-8,gtol=1e-8);tasks,details,clear,selfmm,supportmm,pen=measure(fit.x,True);goal=motor(d.ctrl);gap=min(v for name,v in clear);contact_ok=all(abs(c['face_gap_mm'])<.2 and abs(c['point_lateral_mm'])<22 and -22<c['point_height_mm']<7 for c in details);accepted=bool(contact_ok and gap>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1 and np.all(goal>=lo) and np.all(goal<=hi))
    order=[12,10,11,9] if a.finger=='middle' else [8,6,7,5]
    report=dict(status='sliding_lower_contact_headroom_static_proposal',parameters=vars(a),contacts=details,minimum_upper_gap_mm=gap,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,goal=goal.tolist(),translation_world_mm=fit.x[-3:].tolist(),grasp_finger_motor_order=order,grasp_finger_goal_ticks=goal[np.array(order)-1].tolist(),proposal_accepted=accepted,elapsed_seconds=time.perf_counter()-began,contact_points_allowed_to_slide_in_proposal=True,cube_pinned_only_for_IK=True,free_cube_trajectory_not_validated=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if accepted:
        preload=[];gains=m.actuator_gainprm[:,0]
        if np.all(gains>0) and np.allclose(m.actuator_biasprm[:,1],-gains) and np.allclose(m.actuator_gear[:,0],1):
            for force in (1.5,2.5,3.5):
                control=d.qpos[:16]+pd
                for role,(gid,normal,horizontal) in enumerate(contacts):
                    world=vertices[gid]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid];proj=world@normal;weights=np.exp(-(proj-min(proj))/.0002);point=np.average(world,axis=0,weights=weights);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[gid]));active=finger_active if role==0 else np.arange(12,16);tau=d.qfrc_bias[active]-d.qfrc_passive[active]-(normal@jp[:,active])*force;control[active]=d.qpos[active]+tau/gains[active]
                target=motor(control);valid=bool(np.all(target>=lo) and np.all(target<=hi));preload.append(dict(assumed_each_contact_normal_force_N=force,goal=target.tolist(),native_goal_valid=valid))
        report['model_only_preload_proposals']=preload;np.savez_compressed(out/'endpoint.npz',goal=goal,cube_center=C0+fit.x[-3:]/1000,cube_rotation=B,actual_joint_proposal=d.qpos[:16])
        (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report),flush=True)
if __name__=='__main__':main()
