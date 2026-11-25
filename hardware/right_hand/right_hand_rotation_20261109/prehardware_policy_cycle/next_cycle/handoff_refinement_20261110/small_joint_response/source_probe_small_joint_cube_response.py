"""Local one-joint sensitivity, with matched hold controls and unchanged guards.

These initialized model probes are not physical measurements, SAC-policy trials,
or a general conversion from joint angle to cube error.
"""
from pathlib import Path
import time
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,TICK,SIM_TO_MOTOR_IDS,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/small_joint_response'
    if folder.exists():raise RuntimeError('Preserve previous probes')
    model,initial,_,_,provenance=assets('active_ccd50')
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool=np.load(pool_path,allow_pickle=False)
    pool_review=json.loads((pool_path.parent/'review.json').read_text())
    if sha(pool_path)!=pool_review['state_pool_sha256']:raise RuntimeError('State pool changed')
    folder.mkdir();(folder/'source_probe_small_joint_cube_response.py').write_bytes(Path(__file__).read_bytes())
    samples=[];began=time.perf_counter()
    for index in (1100,3000,3600):
        pos=int(np.flatnonzero(pool['indices']==index)[0]);z=dict(initial)
        z.update({key:pool[key][pos] for key in BOUND});z['time']=pool['time'][pos]
        start=motor(z['ctrl']);results=[]
        def trial(mid=None,delta=0):
            data=restore(model,z);guard=Guard(model);target=start.copy()
            if mid is not None:target[mid-1]+=delta
            if np.any(target<guard.lo) or np.any(target>guard.hi):
                return dict(motor_id=mid,offset_ticks=delta,stop=['native_goal_range'],completed_goals=0),None
            previous=start.copy();stop=[]
            for step in range(17):
                goal=previous+np.clip(target-previous,-2,2)
                if np.max(abs(goal-previous))>2:raise AssertionError('Probe exceeded rate')
                data.ctrl[:]=ticks_to_sim(goal);previous=goal
                for _ in range(75):
                    mujoco.mj_step(model,data);stop=guard.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:
                        stop=list(stop)+['cube_translation_over80mm']
                    if stop:break
                if stop:break
            state={key:getattr(data,key).copy() for key in BOUND}
            B=Rotation.from_quat(data.qpos[-4:][[1,2,3,0]]).as_matrix()
            row=dict(motor_id=mid,offset_ticks=delta,command_offset_deg=delta*TICK*180/np.pi,
                completed_goals=step+1,stop=stop,worst=guard.worst,
                cube_B=B.tolist(),cube_center=data.qpos[-7:-4].tolist())
            return row,state
        baseline,base_state=trial()
        np.savez_compressed(folder/f'state{index}_baseline.npz',**base_state)
        if baseline['stop']:raise RuntimeError('Matched baseline not valid: '+str(baseline))
        for mid in range(1,17):
            for delta in (-7,7):
                row,state=trial(mid,delta)
                row['comparison_valid']=not row['stop'] and row['completed_goals']==17
                if row['comparison_valid']:
                    row['cube_angle_change_vs_matched_hold_deg']=orientation_error_deg(np.array(row['cube_B']),np.array(baseline['cube_B']))
                    row['cube_center_change_vs_matched_hold_mm']=float(np.linalg.norm(np.array(row['cube_center'])-baseline['cube_center'])*1000)
                    sid=list(SIM_TO_MOTOR_IDS).index(mid)
                    row['actual_joint_angle_change_vs_matched_hold_deg']=float(np.degrees(state['qpos'][sid]-base_state['qpos'][sid]))
                results.append(row)
        valid=[r for r in results if r['comparison_valid']]
        errors=[r['cube_angle_change_vs_matched_hold_deg'] for r in valid]
        report=dict(parent_goal_index=index,baseline=baseline,results=results,
            valid_comparisons=len(valid),blocked_comparisons=len(results)-len(valid),
            minimum_cube_angle_change_deg=min(errors) if errors else None,
            median_cube_angle_change_deg=float(np.median(errors)) if errors else None,
            maximum_cube_angle_change_deg=max(errors) if errors else None,
            maximum_response_motor=max(valid,key=lambda r:r['cube_angle_change_vs_matched_hold_deg'])['motor_id'] if valid else None)
        samples.append(report)
        print(json.dumps({k:v for k,v in report.items() if k not in ('baseline','results')}),flush=True)
    all_valid=[r for s in samples for r in s['results'] if r['comparison_valid']]
    result=dict(status='local_model_single_joint_response_probes',model_provenance=provenance,
        state_pool_sha256=sha(pool_path),source_goal_indices=[1100,3000,3600],
        perturbation_target_ticks=[-7,7],perturbation_target_deg=7*TICK*180/np.pi,
        reason_for7ticks='Nearest integer motor target to6.519ticks; this is approximately0.6deg, not an exact0.57deg displacement',
        max_goal_increment_ticks=2,control_seconds=.075,matched_control_goals=17,total_control_seconds=1.275,
        original_native_guards_and_limits_retained=True,other_joint_commands_held=True,
        total_comparisons=96,valid_comparisons=len(all_valid),samples=samples,
        initialized_research_probes_not_uninterrupted_task_success=True,
        scope='Three simulated contact states,17-control-step horizon; no measured real-world average or universal joint-to-cube conversion',
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',result)


if __name__=='__main__':main()
