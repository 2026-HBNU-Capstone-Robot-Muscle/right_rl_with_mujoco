"""Bounded free-cube trials from corrected upper-middle-first transfer.

These are offline sequence comparisons, never a hardware executable. Native
joint, tracking, collision and transport limits remain unchanged. An archived
grasp is a hypothesis; actual contacts and support must be measured anew.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'
sys.path.insert(0,str(WORK))
import mujoco
from native_cycle import assets,restore,Guard,BOUND,motor,ticks_to_sim,sha,save_json
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg,aligned_top_face
from screen_direct_work_grip_20261112 import transition

PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
OUT=PARENT/'corrected_three_finger_sequence'
REFERENCE=WORK/'next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz'

def snapshot(d):
    return {**{k:getattr(d,k).copy() for k in BOUND},'time':float(d.time)}

def simulate(m,d,initial,goals,moving):
    g=Guard(m);cube=m.geom('cube').id;rows=[];trace=[];stop=[]
    count=supported=0;loaded={f:0 for f in ('th','mf','rf')};shift=drift=0.;C0=B0=None
    previous=motor(d.ctrl)
    for i,command in enumerate(goals):
        assert np.max(abs(command-previous))<=2 and np.all(command>=g.lo) and np.all(command<=g.hi)
        d.ctrl[:]=ticks_to_sim(command);previous=command.copy();rows.append(command)
        for ms in range(75):
            mujoco.mj_step(m,d);stop=g.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=moving:
                C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3)
                if C0 is None:C0=C.copy();B0=B.copy()
                load=sample_model(m,d,g.hand,cube);count+=1;supported+=load['supported_sample']
                for f in loaded:
                    force=sum(v['normal_N'] for n,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(n).id])).name or '').startswith(f+'_'))
                    loaded[f]+=force>.1
                shift=max(shift,float(np.linalg.norm(C-C0)*1000));drift=max(drift,orientation_error_deg(B,B0))
            if stop:break
        if (i+1)%100==0 or stop or i+1==len(goals):
            trace.append(dict(goal=i+1,cube_world_mm=(d.geom_xpos[cube]*1000).tolist(),
                              loaded=sample_model(m,d,g.hand,cube)['loaded_hand_geoms'],stop=stop))
        if stop:break
    fractions={k:v/max(count,1) for k,v in loaded.items()}
    passed=bool(not stop and count==4050 and supported/count>.8 and all(v>.8 for v in fractions.values()) and shift<2 and drift<3)
    return dict(done=len(rows),total=len(goals),stop=stop,hold_samples=count,supported_samples=supported,
                contact_fractions=fractions,hold_shift_mm=shift,hold_drift_deg=drift,
                temporary_three_finger_hold_passed=passed,
                top_face=aligned_top_face(d.geom_xmat[cube].reshape(3,3),initial['up']),
                final_load=sample_model(m,d,g.hand,cube),trace=trace,worst=g.worst,terminal_guard=g.last),rows

def main():
    OUT.mkdir(exist_ok=False)
    m,initial,_,_,provenance=assets('active_ccd50')
    reference=dict(np.load(REFERENCE,allow_pickle=False));target=motor(reference['ctrl'])
    sources={
        'before_lower_motion':PARENT/'lower_pad_lift_after_clear_v2/start_state.npz',
        'after_lower_motion':PARENT/'lower_pad_lift_after_clear_v2/end_state.npz'}
    results=[]
    for source_name,source in sources.items():
        saved=dict(np.load(source,allow_pickle=False))
        for order in ('RF_MF_TH','MF_RF_TH'):
            folder=OUT/(source_name+'_'+order);folder.mkdir()
            d=restore(m,saved);start=motor(d.ctrl);goal=target.copy();goal[4:8]=start[4:8]
            moves=transition(start,goal,order);goals=np.vstack([moves,np.tile(goal,(54,1))])
            r,rows=simulate(m,d,initial,goals,len(moves));r.update(source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),order=order)
            np.savez_compressed(folder/'research_goals.npz',start=start,goals=rows)
            np.savez_compressed(folder/'end_state.npz',**snapshot(d));save_json(folder/'review.json',r)
            results.append(r)
            report=dict(rows=results,model_provenance=provenance,reference_sha256=sha(REFERENCE),
                        original_limits_and_guards_unchanged=True,devices_opened=0,
                        final_upper_space_and_next_rotation_certified=False,hardware_execution_allowed=False,physical_success=False)
            save_json(OUT/'review.json',report)
            print(json.dumps(dict(source=source_name,order=order,**{k:r[k] for k in ('done','total','stop','hold_samples','contact_fractions','temporary_three_finger_hold_passed')})),flush=True)

if __name__=='__main__':main()
