"""Small model-only MF preload and thumb tangential lowering corrections."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import lsq_linear
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from servo_MF_temporary_support import mf_tick_increment
from cube_task_frame_20261105 import aligned_top_face,working_rotation,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate

BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=BASE/'lower_three_finger_contact_search/RF_TH_MF/end_state.npz'
OUT=BASE/'lower_three_finger_refinement'

def main():
    OUT.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');d=restore(m,dict(np.load(SOURCE,allow_pickle=False)));g=Guard(m)
    cube=m.geom('cube').id;mf=m.geom('mf_tip').id;th=m.geom('th_tip').id
    face=aligned_top_face(d.geom_xmat[cube].reshape(3,3),initial['up'])['best_candidate_id']
    up=np.asarray(initial['up']);up/=np.linalg.norm(up);start=motor(d.ctrl);previous=start.copy();acc=start.astype(float)
    mf_ids=np.array([11,9,10,8]);th_ids=np.array([3,2,1,0]);th_active=np.arange(12,16);T=2*np.pi/4096;jac=np.zeros((3,m.nv))
    commands=[];trace=[];stop=[];count=supported=clear=0;loaded={f:0 for f in ('mf','rf','th')};C0=B0=None;shift=drift=0.;cert=SweepCertificate(m,g.hand,.3);upper_failure=None;minimum=None
    for i in range(354):
        command=previous.copy()
        if i<300:
            load=sample_model(m,d,g.hand,cube)
            force=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith('mf_'))
            dt,gap=mf_tick_increment(m,d,cube,mf,.3,force);acc[mf_ids]+=dt
            if 150<=i<250:
                points=np.zeros(6);mujoco.mj_geomDistance(m,d,cube,th,.2,points);n=points[3:]-points[:3]
                c=next((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,th}),None)
                if c is not None:n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
                n/=max(np.linalg.norm(n),1e-12)
                direction=-up+n*(up@n);direction/=max(np.linalg.norm(direction),1e-12)
                mujoco.mj_jac(m,d,jac,None,points[3:],int(m.geom_bodyid[th]));A=jac[:,th_active]*T*1000
                lo=np.maximum(-2,np.ceil(g.lo[th_ids])-acc[th_ids]);hi=np.minimum(2,np.floor(g.hi[th_ids])-acc[th_ids])
                dt_th=lsq_linear(np.vstack([A,.015*np.eye(4)]),np.r_[direction*.015,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
                acc[th_ids]+=dt_th
            for ids in (mf_ids,th_ids):
                acc[ids]=np.clip(acc[ids],np.ceil(g.lo[ids]),np.floor(g.hi[ids]));command[ids]=np.rint(acc[ids]).astype(int)
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
        d.ctrl[:]=ticks_to_sim(command);commands.append(command);previous=command.copy()
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=300:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:C0=C.copy();B0=B.copy()
                load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                for f in loaded:loaded[f]+=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if upper_failure is None:
                    lower,bad=cert.query(d,C,working_rotation(B,face));minimum=lower if minimum is None else min(minimum,lower)
                    if bad or lower<=.3:upper_failure=dict(sample=count,gap_mm=lower,geoms=[m.geom(int(cert.ids[j])).name for j in bad])
                    else:clear+=1
            if stop:break
        if (i+1)%100==0 or stop or i+1==354:
            load=sample_model(m,d,g.hand,cube);forces={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_')) for f in loaded}
            trace.append(dict(goal=i+1,stop=stop,forces_N=forces,top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)))
            print(json.dumps(trace[-1]),flush=True)
        if stop:break
    fractions={f:v/max(count,1) for f,v in loaded.items()};top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)
    passed=bool(not stop and count==clear==4050 and supported/count>.8 and all(v>.8 for v in fractions.values()) and shift<2 and drift<3 and top['face_id']==face)
    r=dict(done=len(commands),total=354,stop=stop,source_sha256=sha(SOURCE),source_face=face,hold_samples=count,supported_samples=supported,
           loaded_fractions=fractions,hold_shift_mm=shift,hold_drift_deg=drift,upper_certified_samples=clear,upper_first_failure=upper_failure,
           minimum_upper_mm=minimum,final_top=top,three_finger_lower_clear_hold_passed=passed,trace=trace,worst=g.worst,terminal_guard=g.last,
           model_provenance=provenance,MF_model_preload_target_N=.3,thumb_requested_tangential_lowering_mm=1.5,
           uses_exact_model_distances_normals_and_forces=True,force_targets_not_calibrated_to_real_current=True,
           native_guards_and_ranges_unchanged=True,new_SAC_training_steps=0,devices_opened=0,physical_success=False,hardware_execution_allowed=False)
    np.savez_compressed(OUT/'research_goals.npz',start=start,goals=commands);np.savez_compressed(OUT/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(OUT/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('trace','worst','terminal_guard','model_provenance')}),flush=True)

if __name__=='__main__':main()
