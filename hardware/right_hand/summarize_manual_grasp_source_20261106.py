"""Extract the already executed grasp/hold prefix, without OPEN or devices."""
import hashlib,json,sys
from pathlib import Path
from collections import Counter
import numpy as np
from natural_cradle_grasp_plan_20261030 import validate_candidate,LOADED
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261106/manual_grasp'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    sys.stdout.reconfigure(encoding='utf-8');source=R/'natural_cradle_grasp_20261106_175553_695333/result.json';r=json.loads(source.read_bytes());assert r['status']=='natural_cradle_grasp_observation_complete' and r['settings_restored'] and r['all_torque_off_verified'];z=validate_candidate();cut=int(np.flatnonzero(z['phases']=='stage_6_grasp_hold')[-1])+1;rows=z['goals'][:cut];phases=z['phases'][:cut];assert list(dict.fromkeys(phases))==['stage_5_loaded_grasp','stage_6_grasp_hold'];assert np.array_equal(rows[-67:],np.tile(rows[-1],(67,1)));assert np.max(abs(np.diff(np.vstack([z['start'],rows]),axis=0)))<=2
    O.mkdir(parents=True,exist_ok=True);np.savez_compressed(O/'candidate.npz',start=z['start'],goals=rows,phases=phases)
    selected={k:r.get(k) for k in ('timestamp','status','failed_phase','error','all_torque_off_verified','settings_restored','cube_removal_confirmed','grasp_position_ticks','grasp_goal_ticks','placement_position_ticks','placement_goal_ticks','open_before_cube_photo','placement_photo','grasp_observation_photo','opened_before_remove_photo')}
    selected.update(status='executed_grasp_prefix_extracted_no_automatic_open',source_physical_result=source.relative_to(R).as_posix(),source_physical_result_sha256=sha(source),source_candidate=LOADED.relative_to(R).as_posix(),source_candidate_sha256=sha(LOADED),candidate_sha256=sha(O/'candidate.npz'),loaded_goals=len(rows),loaded_seconds=len(rows)*.075,source_phases=dict(Counter(z['phases'].tolist())),prefix_phases=dict(Counter(phases.tolist())),new_joint_targets_added=False,automatic_open=False,rotation_goals=0,source_run_torque_off_verified=True,source_run_settings_restored=True,devices_opened=False,assistant_motor_writes=0,manual_correction_not_recorded_yet=True)
    (O/'source_review.json').write_text(json.dumps(selected,indent=2,ensure_ascii=False)+'\n',encoding='utf-8');print(json.dumps(selected,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
