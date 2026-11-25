"""Replay placement -> rotation -> corrected transfer -> TH/MF/RF hold.

The cube is initialized once. All later segments are commands only. This
report is offline evidence, and does not authorize a physical execution.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,ticks_to_sim,motor,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,face_up_error_deg,aligned_top_face,working_rotation
from upper_layer_separation_certificate_20261105 import SweepCertificate
PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mu',type=float,choices=(.2,.3),default=.3);args=parser.parse_args()
    out=PARENT/f'corrected_three_finger_continuous_mu{args.mu:g}';out.mkdir(exist_ok=False)
    segments=[(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',None),
              (PARENT/'middle_shape_like_index/research_goals.npz',None),
              (PARENT/'clear_middle_before_push/middle_shape_like_index/research_goals.npz',150),
              (PARENT/'lower_pad_lift_after_clear_v2/research_goals.npz',None),
              (PARENT/'reached_lower_contact_search/RF_TH_MF/research_goals.npz',None)]
    parts=[];manifest=[];previous=None
    for path,limit in segments:
        z=dict(np.load(path,allow_pickle=False));goals=z['goals'] if limit is None else z['goals'][:limit]
        if previous is not None:assert np.array_equal(previous,z['start']),(path,'discontinuous source goals')
        else:start=z['start'].copy()
        previous=goals[-1].copy();parts.append(goals)
        manifest.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),goals=len(goals)))
    q=np.vstack(parts);assert np.max(abs(np.diff(np.vstack([start,q]),axis=0)))<=2
    np.savez_compressed(out/'candidate.npz',start=start,goals=q)
    m,initial,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=args.mu;d=restore(m,initial);g=Guard(m)
    assert np.array_equal(start,motor(d.ctrl)) and np.all(q>=g.lo) and np.all(q<=g.hi)
    cube=m.geom('cube').id;counts=0;supported=0;loaded={f:0 for f in ('th','mf','rf')};C0=B0=None
    shift=drift=0.;checks=0;stop=[];trace=[];certs={};boundaries=np.cumsum([len(x) for x in parts]);face_id=None
    for i,command in enumerate(q):
        d.ctrl[:]=ticks_to_sim(command)
        for _ in range(75):
            mujoco.mj_step(m,d);checks+=1;stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=len(q)-54:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:
                    C0=C.copy();B0=B.copy();face_id=aligned_top_face(B,initial['up'])['best_candidate_id']
                    for name in ('original_face5','nearest_up_face'):
                        certs[name]=dict(certificate=SweepCertificate(m,g.hand,.3),samples=0,minimum=None,first_failure=None)
                load=sample_model(m,d,g.hand,cube);counts+=1;supported+=load['supported_sample']
                for f in loaded:
                    force=sum(v['normal_N'] for n,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(n).id])).name or '').startswith(f+'_'))
                    loaded[f]+=force>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
                for name,r in certs.items():
                    if r['first_failure'] is not None:continue
                    basis=B if name=='original_face5' else working_rotation(B,face_id)
                    lower,bad=r['certificate'].query(d,C,basis)
                    r['minimum']=lower if r['minimum'] is None else min(r['minimum'],lower)
                    if bad or lower<=.3:
                        r['first_failure']=dict(hold_sample=counts,gap_mm=lower,geoms=[m.geom(int(r['certificate'].ids[j])).name for j in bad])
                    else:r['samples']+=1
            if stop:break
        if i+1 in boundaries or stop:
            row=dict(goal=i+1,stop=stop,cube_world_mm=(d.geom_xpos[cube]*1000).tolist(),
                     top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),initial['up']),
                     loaded=sample_model(m,d,g.hand,cube)['loaded_hand_geoms'])
            trace.append(row);print(json.dumps(dict(mu=args.mu,**row)),flush=True)
        if stop:break
    fractions={f:v/max(1,counts) for f,v in loaded.items()}
    hold=bool(not stop and i+1==len(q) and counts==4050 and supported/counts>.8 and all(v>.8 for v in fractions.values()) and shift<2 and drift<3)
    certificates={n:{k:v for k,v in r.items() if k!='certificate'} for n,r in certs.items()}
    report=dict(status='one_initialization_corrected_three_finger_chain',mu_hypothesis=args.mu,done=i+1,total=len(q),
                dynamic_samples=checks,stop=stop,continuous_three_finger_supported_hold_passed=hold,
                hold_samples=counts,supported_samples=supported,loaded_fractions=fractions,hold_shift_mm=shift,hold_drift_deg=drift,
                upper_certificates=certificates,nearest_up_face_id=face_id,
                original_face5_tilt_deg=face_up_error_deg(d.geom_xmat[cube].reshape(3,3),5,initial['up']),
                final_top=aligned_top_face(d.geom_xmat[cube].reshape(3,3),initial['up']),
                trace=trace,worst=g.worst,terminal_guard=g.last,segments=manifest,model_provenance=provenance,
                candidate_sha256=sha(out/'candidate.npz'),state_resets_after_initialization=0,
                original_ranges_geometries_and_guards_unchanged=True,source_first1417_actual_observation_not_full_chain_physical_success=True,
                final_left_alignment_verified=False,next_rotation_included=False,new_SAC_training_steps=0,
                physical_success=False,hardware_execution_allowed=False,devices_opened=0)
    np.savez_compressed(out/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time)
    save_json(out/'review.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','worst','terminal_guard','segments','model_provenance')}),flush=True)

if __name__=='__main__':main()
