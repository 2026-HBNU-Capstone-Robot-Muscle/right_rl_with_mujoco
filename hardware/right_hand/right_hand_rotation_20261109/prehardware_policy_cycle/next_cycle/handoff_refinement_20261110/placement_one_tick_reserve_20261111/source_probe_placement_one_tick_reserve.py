"""Test small command reserves at the saved1mm-placement failure checkpoint.

These are finite open-loop comparisons from an actually replayed state, not a
new robust policy or permission to alter native joint/collision thresholds.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg

def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'placement_one_tick_reserve_20261111';source=base/'robust_cycle_preparation_20261111/failure_checkpoints/initial_cube_worldX_plus1mm/state1700.npz';reference=base/'initial_second_turn_hold_replay/research_goals.npz'
    if folder.exists():raise RuntimeError('Preserve finite placement comparisons')
    goals=np.load(reference,allow_pickle=False)['goals'][1700:2300];z=dict(np.load(source,allow_pickle=False));folder.mkdir();(folder/'source_probe_placement_one_tick_reserve.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for mid,offset in ((11,-1),(11,1),(9,-1),(9,1)):
        m,initial,*_=assets('active_ccd50');d=restore(m,z);g=Guard(m);previous=motor(d.ctrl);commands=[];stop=[];label=f'ID{mid}_bias{offset:+d}';sub=folder/label;sub.mkdir()
        for i,nominal in enumerate(goals):
            desired=nominal.copy();desired[mid-1]+=offset;command=previous+np.clip(desired-previous,-2,2)
            if np.any(command<g.lo) or np.any(command>g.hi):stop=['native_goal_range'];break
            d.ctrl[:]=ticks_to_sim(command);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(m,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if stop:break
            if stop:break
        target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90);cube=m.geom('cube').id
        np.savez_compressed(sub/'research_goals.npz',start=motor(z['ctrl']),goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        r=dict(case=label,path=sub.relative_to(OUT).as_posix(),motor_id=mid,offset_ticks=offset,steps=len(commands),after_global_goal=1700+len(commands),stop=stop,checkpoint90_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),target),passed_local600_goals=bool(not stop and len(commands)==600),worst=g.worst,terminal_guard=g.last);rows.append(r);save_json(sub/'review.json',r);save_json(folder/'progress.json',dict(rows=rows));print(json.dumps({k:v for k,v in r.items() if k not in ('terminal_guard','worst')}),flush=True)
    save_json(folder/'review.json',dict(status='finite1tick_MF_reserves_from_actual_1mm_placement_checkpoint',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),reference=reference.relative_to(OUT).as_posix(),reference_sha256=sha(reference),rows=rows,native_guards_and_joint_limits_unchanged=True,full_two_turn_hold_cycle_not_yet_evaluated=True,original_failure_not_reclassified=True,actual_state_resets_after_checkpoint=0,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))

if __name__=='__main__':main()
