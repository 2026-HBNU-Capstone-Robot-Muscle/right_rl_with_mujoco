"""Model-only tangential thumb lowering while ring and lower pads support cube."""
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
from cube_task_frame_20261105 import aligned_top_face,working_rotation,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
BASE=ROOT/'right_hand_rotation_20261112/support_first_transfer'
SOURCE=BASE/'sequential_low_mu_v4/end_state.npz';OUT=BASE/'low_mu_thumb_lowering_v1'

def main():
    OUT.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.2
    d=restore(m,dict(np.load(SOURCE,allow_pickle=False)));g=Guard(m);cube=m.geom('cube').id
    up=initial['up']/np.linalg.norm(initial['up']);face=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)['best_candidate_id']
    start=motor(d.ctrl);previous=start.copy();acc=start.astype(float);T=2*np.pi/4096;rows=[];trace=[];stop=[]
    count=supported=clear=0;loaded={f:0 for f in ('rf','mf','th')};C0=B0=None;shift=drift=0.;cert=SweepCertificate(m,g.hand,.3);upper_failure=None;minimum=None
    for i in range(454):
        command=previous.copy();load=sample_model(m,d,g.hand,cube)
        if i<400:
            for f,active,ids in (('rf',np.arange(8,12),np.array([15,13,14,12])),('th',np.arange(12,16),np.array([3,2,1,0]))):
                tip=m.geom(f+'_tip').id;points=np.zeros(6);mujoco.mj_geomDistance(m,d,cube,tip,.5,points)
                n=points[3:]-points[:3];c=next((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),None)
                if c is not None:n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
                if np.linalg.norm(n)<1e-10:n=d.geom_xpos[tip]-d.geom_xpos[cube]
                n/=np.linalg.norm(n)
                force=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))
                delta=-n*float(np.clip((.3-force)*.1,-.025,.025))
                if f=='th':
                    tangent=-up+n*(up@n);tangent/=max(np.linalg.norm(tangent),1e-12);delta+=tangent*.02
                J=np.zeros((3,m.nv));mujoco.mj_jac(m,d,J,None,points[3:],int(m.geom_bodyid[tip]));A=J[:,active]*T*1000
                lo=np.maximum(-2,np.ceil(g.lo[ids])-acc[ids]);hi=np.minimum(2,np.floor(g.hi[ids])-acc[ids])
                dt=lsq_linear(np.vstack([A,.015*np.eye(4)]),np.r_[delta,np.zeros(4)],bounds=(lo-1e-9,hi+1e-9)).x
                acc[ids]=np.clip(acc[ids]+dt,np.ceil(g.lo[ids]),np.floor(g.hi[ids]));command[ids]=np.rint(acc[ids]).astype(int)
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
        previous=command.copy();rows.append(command);d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=400:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:C0=C.copy();B0=B.copy()
                load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                for f in loaded:loaded[f]+=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if upper_failure is None:
                    gap,bad=cert.query(d,C,working_rotation(B,face));minimum=gap if minimum is None else min(minimum,gap)
                    if bad or gap<=.3:upper_failure=dict(sample=count,gap_mm=gap,geoms=[m.geom(int(cert.ids[j])).name for j in bad])
                    else:clear+=1
            if stop:break
        if (i+1)%100==0 or stop or i+1==454:
            trace.append(dict(goal=i+1,stop=stop,loaded=sample_model(m,d,g.hand,cube)['loaded_hand_geoms'],top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)))
            print(json.dumps(trace[-1]),flush=True)
        if stop:break
    fractions={f:v/max(1,count) for f,v in loaded.items()};top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up)
    passed=bool(not stop and count==clear==4050 and supported/count>.8 and all(v>.8 for v in fractions.values()) and shift<2 and drift<3 and top['face_id']==face)
    r=dict(passed=passed,done=len(rows),total=454,stop=stop,mu_hypothesis=.2,hold_samples=count,supported_samples=supported,
           loaded_fractions=fractions,upper_certified_samples=clear,upper_first_failure=upper_failure,minimum_upper_mm=minimum,
           hold_shift_mm=shift,hold_drift_deg=drift,final_top=top,source_sha256=sha(SOURCE),model_provenance=provenance,
           trace=trace,worst=g.worst,terminal_guard=g.last,model_contact_feedback_not_hardware_sensors=True,
           native_guards_and_limits_unchanged=True,devices_opened=0,physical_success=False,hardware_execution_allowed=False)
    np.savez_compressed(OUT/'research_goals.npz',start=start,goals=rows);np.savez_compressed(OUT/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(OUT/'review.json',r);print(json.dumps({k:v for k,v in r.items() if k not in ('trace','worst','terminal_guard','model_provenance')}),flush=True)

if __name__=='__main__':main()
