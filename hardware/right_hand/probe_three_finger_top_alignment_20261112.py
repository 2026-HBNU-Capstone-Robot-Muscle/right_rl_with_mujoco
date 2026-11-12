"""Attempt a small top-face alignment with three model contacts and free cube.

Privileged model geometry supplies contact anchors. This is an offline teacher,
not SAC training, a real sensor controller, or a hardware authorization.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import normal,face_up_error_deg,orientation_error_deg,aligned_top_face,working_rotation
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'
OUT=BASE/'three_finger_top_alignment'
SOURCE=BASE/'reached_lower_contact_search/RF_TH_MF/end_state.npz'

def main():
    OUT.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');saved=dict(np.load(SOURCE,allow_pickle=False));d=restore(m,saved);g=Guard(m)
    cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    up=np.asarray(initial['up']);up/=np.linalg.norm(up);face=aligned_top_face(B,up)['best_candidate_id'];n=B@normal(face)
    cross=np.cross(n,up);length=np.linalg.norm(cross);angle=float(np.arccos(np.clip(n@up,-1,1)))
    axis=cross/length if length>1e-10 else np.array([1.,0.,0.])
    anchors=[]
    for f,active,ids in (('rf',np.arange(8,12),np.array([15,13,14,12])),('th',np.arange(12,16),np.array([3,2,1,0])),('mf',np.arange(4,8),np.array([11,9,10,8]))):
        tip=m.geom(f+'_tip').id;contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
        if not contacts:raise RuntimeError('Source no longer has every fingertip contact')
        c=min(contacts,key=lambda x:x.dist);G=d.geom_xmat[tip].reshape(3,3)
        anchors.append((f,tip,active,ids,G.T@(c.pos-d.geom_xpos[tip]),B.T@(c.pos-C)))
    start=motor(d.ctrl);previous=start.copy();acc=start.astype(float);jac=np.zeros((3,m.nv));T=2*np.pi/4096
    rows=[];trace=[];stop=[];count=supported=0;loaded={f:0 for f in ('th','mf','rf')};shift=drift=0.;C0=B0=None
    moving=max(100,int(np.ceil(np.degrees(angle)/.03)))+100;cert=SweepCertificate(m,g.hand,.3);upper_failure=None;upper_clear=0;minimum=None
    for i in range(moving+54):
        command=previous.copy()
        if i<moving:
            desired_B=Rotation.from_rotvec(axis*angle*min(1.,(i+1)/(moving-100))).as_matrix()@B
            for f,tip,active,ids,local,anchor in anchors:
                G=d.geom_xmat[tip].reshape(3,3);point=d.geom_xpos[tip]+G@local;target=C+desired_B@anchor
                velocity=np.clip((target-point)*1000*.12,-.2,.2)
                mujoco.mj_jac(m,d,jac,None,point,int(m.geom_bodyid[tip]));A=jac[:,active]*T*1000
                lo=np.maximum(-2,np.ceil(g.lo[ids])-acc[ids]);hi=np.minimum(2,np.floor(g.hi[ids])-acc[ids])
                step=lsq_linear(np.vstack([A,.025*np.eye(4)]),np.r_[velocity,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
                acc[ids]=np.clip(acc[ids]+step,np.ceil(g.lo[ids]),np.floor(g.hi[ids]));command[ids]=np.rint(acc[ids]).astype(int)
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
        previous=command.copy();rows.append(command);d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=moving:
                curC=d.geom_xpos[cube];curB=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:C0=curC.copy();B0=curB.copy()
                load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                for f in loaded:loaded[f]+=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))>.1
                shift=max(shift,float(np.linalg.norm(curC-C0)*1000));drift=max(drift,orientation_error_deg(curB,B0))
                if upper_failure is None:
                    gap,bad=cert.query(d,curC,working_rotation(curB,face));minimum=gap if minimum is None else min(minimum,gap)
                    if bad or gap<=.3:upper_failure=dict(sample=count,geoms=[m.geom(int(cert.ids[j])).name for j in bad],gap_mm=gap)
                    else:upper_clear+=1
            if stop:break
        if (i+1)%150==0 or stop or i+1==moving+54:
            trace.append(dict(goal=i+1,top_face_tilt_deg=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),face,up),stop=stop,
                              loaded=sample_model(m,d,g.hand,cube)['loaded_hand_geoms']))
            print(json.dumps(trace[-1]),flush=True)
        if stop:break
    tilt=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),face,up)
    passed=bool(not stop and count==upper_clear==4050 and supported/count>.8 and all(v/count>.8 for v in loaded.values()) and tilt<5 and shift<2 and drift<3)
    r=dict(source_sha256=sha(SOURCE),model_provenance=provenance,source_selected_face=face,source_tilt_deg=float(np.degrees(angle)),
           selected_face_is_nearest_up_face_not_proof_of_requested_cube_orientation=True,done=len(rows),total=moving+54,stop=stop,
           hold_samples=count,supported_samples=supported,loaded_fractions={f:v/max(1,count) for f,v in loaded.items()},
           hold_shift_mm=shift,hold_drift_deg=drift,final_tilt_deg=tilt,upper_certified_samples=upper_clear,upper_first_failure=upper_failure,
           minimum_upper_mm=minimum,three_finger_level_clear_hold_passed=passed,trace=trace,worst=g.worst,terminal_guard=g.last,
           uses_exact_model_cube_contact_feedback=True,new_SAC_training_steps=0,native_guards_and_limits_unchanged=True,
           devices_opened=0,physical_success=False,hardware_execution_allowed=False,next_whole_cube_rotation_included=False)
    np.savez_compressed(OUT/'research_goals.npz',start=start,goals=rows);np.savez_compressed(OUT/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(OUT/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('model_provenance','trace','worst','terminal_guard')}),flush=True)

if __name__=='__main__':main()
