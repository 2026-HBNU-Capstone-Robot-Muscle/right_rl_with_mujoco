"""Offline probe only: taught thumb values with recorded pitch-end supports."""
from pathlib import Path
import json,hashlib
import numpy as np,mujoco
from build_taught_palm_transfer_20261118 import interpolate
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/vertical_direct_thumb_return'

def main():
    ref=json.loads((BASE/'thumb_push_reference.json').read_bytes())
    if hashlib.sha256((ROOT/ref['source']).read_bytes()).hexdigest()!=ref['source_sha256']:
        raise RuntimeError('Thumb source changed')
    old=np.load(ROOT/'right_hand_rotation_20261118/vertical_cycle/candidate.npz',allow_pickle=False)
    start=old['goals'][-1].copy();pushed=start.copy();pushed[:4]=ref['holding_goal_ticks']
    rows=interpolate(start,pushed);validate_rows(start,rows)
    file=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':file.read_bytes()})
    d=mujoco.MjData(m);check=inspect_for(m)[2]
    def depths(goals):
        result=[{},{}]
        for q in goals:
            d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d)
            for dst,values in zip(result,check(d)):
                for pair,value in values.items():dst[pair]=max(dst.get(pair,0.),float(value))
        return result
    baseline=depths(old['goals']);probe=depths(rows)
    added=[{p:v-baseline[i].get(p,0.) for p,v in data.items() if v>baseline[i].get(p,0.)+1e-9}
           for i,data in enumerate(probe)]
    result=dict(kind='thumb_only_direct_return_offline_probe',start_ticks=start.tolist(),
        target_ticks=pushed.tolist(),thumb_source=ref['source'],goals=len(rows),
        other12_fixed_at_recorded_vertical_end=bool(np.all(rows[:,4:]==start[4:])),
        maximum_step_ticks=int(np.max(abs(np.diff(np.vstack([start,rows]),axis=0)))),
        baseline_support_mm=baseline[0],baseline_self_mm=baseline[1],
        probe_support_mm=probe[0],probe_self_mm=probe[1],
        added_support_mm=added[0],added_self_mm=added[1],
        colliders_unchanged=True,cube_motion_evaluated=False,
        clearance_waypoint_still_required=True,physical_execution=False,
        executable_candidate_created=False)
    (BASE/'thumb_push_probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    # A thumb-only replay is not a valid inference when the observed support
    # fingers differ from the recorded pitch end. Compare an existing receiving
    # shape without copying the new full-hand measurement into motor goals.
    known_path=ROOT/'right_hand_rotation_20261118/vertical_return/taught_palm_121158.json'
    known=json.loads(known_path.read_bytes());lowered=start.copy()
    lowered[4:12]=np.asarray(known['goal_ticks'])[4:12]
    lower_rows=interpolate(start,lowered)
    lower_pushed=lowered.copy();lower_pushed[:4]=ref['holding_goal_ticks']
    push_rows=interpolate(lowered,lower_pushed)
    combined=np.vstack([lower_rows,push_rows]);validate_rows(start,combined)
    support_probe=depths(combined)
    raw=json.loads((ROOT/ref['source']).read_bytes());measured=depths(np.array([raw['snapshots'][-1]['goal']]))
    comparison=dict(kind='receiving_support_comparison_not_executable',
        source_receiving=known_path.relative_to(ROOT).as_posix(),
        source_receiving_sha256=hashlib.sha256(known_path.read_bytes()).hexdigest(),
        lower_index_middle_goals=len(lower_rows),thumb_push_goals=len(push_rows),
        lower_target_ticks=lowered.tolist(),push_target_ticks=lower_pushed.tolist(),
        thumb_held_during_support_lowering=True,ring_goals_preserved=True,
        support_mm=support_probe[0],self_mm=support_probe[1],
        observed_full_hand_model_support_mm=measured[0],observed_full_hand_model_self_mm=measured[1],
        observed_full_hand_not_admitted_as_new_target=True,
        cube_motion_evaluated=False,physical_execution=False,executable_candidate_created=False)
    (BASE/'receiving_support_probe.json').write_text(json.dumps(comparison,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(comparison,ensure_ascii=False))

if __name__=='__main__':main()
