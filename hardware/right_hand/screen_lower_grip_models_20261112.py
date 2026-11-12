"""Source-bound new-path finite-gain and hand-only checks, every executed1ms."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,ticks_to_sim,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate
from lower_grip_trial_plan_20261112 import OUT,CANDIDATE,candidate,sha

def main():
    p=argparse.ArgumentParser();p.add_argument('--gain',type=float,choices=(.5,1.,1.5),required=True);p.add_argument('--hand-only',action='store_true');a=p.parse_args()
    folder=OUT/'models';folder.mkdir(exist_ok=True);output=folder/f'{"hand" if a.hand_only else "loaded"}_gain{a.gain:g}.json'
    if output.exists():raise RuntimeError('Preserve source-bound model result')
    z=candidate();q=z['goals'];rows=[]
    for mu in ((.3,) if a.hand_only else (.2,.3)):
        m,initial,_,_,provenance=assets('active_ccd50');m.actuator_gainprm[:,0]*=a.gain;m.actuator_biasprm[:,1]*=a.gain;m.geom_friction[:,0]=mu
        if a.hand_only:
            mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
        d=restore(m,initial);g=Guard(m);cube=m.geom('cube').id;cert=None if a.hand_only else SweepCertificate(m,g.hand,.3)
        count=supported=clear=0;minimum=float('inf');shift=drift=0.;checks=0;C0=B0=None;stop=[];task_failure=[]
        finger_loaded={f:0 for f in ('th','mf','rf')};working_face=None;upper_failure=None
        for i,command in enumerate(q):
            d.ctrl[:]=ticks_to_sim(command)
            for _ in range(75):
                mujoco.mj_step(m,d);checks+=1;allstop=g.check(d)
                stop=([r for r in allstop if r in ('self_mm','support_mm','tracking_rad','range_rad','nonfinite_state')] if a.hand_only else allstop)
                if not a.hand_only and np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if not a.hand_only and i>=len(q)-54:
                    C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                    if C0 is None:
                        C0=C.copy();B0=B.copy()
                        from cube_task_frame_20261105 import aligned_top_face,working_rotation
                        working_face=aligned_top_face(B,initial['up'])['best_candidate_id']
                    count+=1;load=sample_model(m,d,g.hand,cube);supported+=load['supported_sample']
                    for f in finger_loaded:finger_loaded[f]+=sum(v['normal_N'] for name,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(name).id])).name or '').startswith(f+'_'))>.1
                    if upper_failure is None:
                        lower,bad=cert.query(d,C,working_rotation(B,working_face));minimum=min(minimum,lower);clear+=not bad and lower>.3
                        if bad or lower<=.3:upper_failure=dict(sample=count,gap_mm=lower,geoms=[m.geom(int(cert.ids[j])).name for j in bad])
                    shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                if stop or task_failure:break
            if stop or task_failure:break
        tilt=None if a.hand_only else face_up_error_deg(d.geom_xmat[cube].reshape(3,3),working_face if working_face is not None else 5,initial['up'])
        passed=bool(not stop and i+1==len(q) and (a.hand_only or (count==4050 and supported/4050>.8 and shift<2 and drift<3)))
        row=dict(gain=a.gain,mu=mu,passed=passed,done=i+1,total=len(q),all1ms_samples=checks,stop=stop,task_failure=task_failure,
            hold_samples=count,supported_samples=supported,upper_certified_samples=clear,minimum_upper_mm=minimum if count else None,
            hold_shift_mm=shift,hold_drift_deg=drift,tilt_deg=tilt,worst=g.worst,terminal_guard=g.last,
            finger_loaded_fractions={f:v/max(1,count) for f,v in finger_loaded.items()},upper_first_failure=upper_failure,
            scope='Bounded supervised grasp observation; no upper-layer or next-rotation command',
            final_work_grasp_task_passed=bool(passed and count==clear==4050 and tilt is not None and tilt<5 and all(v/count>.8 for v in finger_loaded.values())))
        rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(output,dict(status='support_first_source_bound_finite_model_screen',candidate_sha256=sha(CANDIDATE),
        checker_sha256=sha(Path(__file__)),gain=a.gain,hand_only=a.hand_only,rows=rows,every_executed_ms_checked=1,
        native_guards_changed=False,actual_state_resets_after_initial_placement=0,model_provenance=provenance,
        physical_success=False,devices_opened=0,hardware_execution_allowed=False))

if __name__=='__main__':main()
