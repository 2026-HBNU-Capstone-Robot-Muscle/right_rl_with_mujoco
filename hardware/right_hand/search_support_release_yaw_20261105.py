"""Allow the supporting index root to move during a free-cube extension.

The previous8-joint search held that support fixed. This compares a genuinely
different contact sequence, with the same original dynamic and native guards.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R,CP,ramp
from research_rotation_body_hold_20261104 import evaluate,restore,motor,native

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--index-delta',type=int,choices=(-64,-96,-128),required=True);a=ap.parse_args();out=ROOT/f'support_release_search/index{a.index_delta}';out.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');start=motor(z['ctrl']);lo,hi,_,_=native(m);old=np.load(R/'right_hand_rotation_20261104/forward_hold/candidate.npz');cases=[];best=None;sources={}
    for seed in (23,37):
        source=ROOT/f'free_extension_search/seed{seed}/review.json';raw=json.loads(source.read_bytes());sources[str(source.relative_to(R))]=hashlib.sha256(source.read_bytes()).hexdigest();base=raw['cases'][raw['best_index']]['deltas_by_motor']
        for scale in (1.,1.5,2.):
            began=time.perf_counter();squeeze=start.copy();squeeze[0]+=12;released=squeeze.copy();released[7]+=a.index_delta;goal=released.copy()
            for mid,value in base.items():
                index=int(mid)-1
                # The thumb root keeps the previously tested delta; other
                # samples may grow. Illegal proposals are skipped, not clipped.
                delta=int(value) if index==3 else int(np.rint(value*scale))
                goal[index]=start[index]+delta+(12 if index==0 else 0)
            if any(np.any(g<lo) or np.any(g>hi) for g in (squeeze,released,goal)):
                cases.append(dict(parent_seed=seed,scale=scale,skipped_native_bound=True));continue
            rows=np.vstack([ramp(start,squeeze,27),ramp(squeeze,released,27),ramp(released,goal,67)])
            assert np.max(abs(np.diff(np.vstack([start,rows]),axis=0)))<=2
            temporary=out/'latest_trial_end.npz';d=evaluate(m,z,rows,temporary);fd=restore(m,np.load(temporary));B=fd.xmat[m.body('cube').id].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@z['up'],-1,1))));error=abs((d['whole_yaw_deg']-90+180)%360-180);score=error+3*max(0,tilt-5)+2*max(0,d['hold_drift_deg']-3)+2*max(0,d['hold_shift_mm']-2)
            if not d['safety_passed']:score=1000+100*(1-d['samples_completed']/len(rows))
            passed=bool(d['safety_passed'] and error<5 and tilt<5 and d['hold_evaluated_steps']==4050 and d['hold_drift_deg']<3 and d['hold_shift_mm']<2)
            row=dict(parent_seed=seed,scale=scale,index_delta_ticks=a.index_delta,goal=goal.tolist(),score=score,target90_filter_passed=passed,final_up_tilt_deg=tilt,dynamic=d,elapsed_seconds=time.perf_counter()-began);cases.append(row)
            if best is None or score<best['score']:
                best=row;(out/'best_end_state.npz').write_bytes(temporary.read_bytes());np.savez_compressed(out/'best_suffix.npz',start=start,goals=rows);np.savez_compressed(out/'candidate.npz',start=old['start'],goals=np.vstack([old['goals'],rows]),phases=np.r_[old['phases'],['experimental_support_release_yaw']*len(rows)])
            report=dict(status='index_support_release_then_free_cube_yaw_extension',index_delta_ticks=a.index_delta,source_search_sha256=sources.copy(),cases=cases,best_score=None if best is None else best['score'],best_target90_passed=False if best is None else best['target90_filter_passed'],full_initial_placement_not_replayed=True,upper_layer_clearance_and_common_grip_not_validated=True,all_original_dynamic_guards_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
            (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(index=a.index_delta,parent_seed=seed,scale=scale,safety=d['safety_passed'],yaw=d['whole_yaw_deg'],tilt=tilt,score=score,target90=passed)),flush=True)
if __name__=='__main__':main()
