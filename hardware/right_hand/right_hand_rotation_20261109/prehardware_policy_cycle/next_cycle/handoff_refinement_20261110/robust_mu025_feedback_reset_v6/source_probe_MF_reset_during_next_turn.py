"""Reposition MF between pushes while the other hand surfaces support the cube."""
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha,angle
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='actual_A_next_MF_reset376');p.add_argument('--source-state');p.add_argument('--sliding-friction',type=float);a=p.parse_args()
    if not a.tag.replace('_','').isalnum():raise ValueError('Local reset tag required')
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/a.tag;source=base/'actual_A_next_checkpoint376/state.npz'
    if a.source_state:
        source=(OUT/a.source_state).resolve()
        if not source.is_relative_to(OUT.resolve()):raise ValueError('Local actual source required')
    if folder.exists():raise RuntimeError('Preserve MF reset study')
    model,initial,_,_,provenance=assets('active_ccd50')
    if a.sliding_friction is not None:
        if not .2<=a.sliding_friction<=.4:raise ValueError('Bounded offline friction hypothesis required')
        model.geom_friction[:,0]=a.sliding_friction
    z=dict(np.load(source,allow_pickle=False));origin=restore(model,z);cube=model.geom('cube').id;C0=origin.geom_xpos[cube].copy();B0=origin.geom_xmat[cube].reshape(3,3).copy();sid=np.asarray(SIM_TO_MOTOR_IDS)-1;active=np.arange(4,8)
    folder.mkdir();(folder/'source_probe_MF_reset_during_next_turn.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for pip,dip,mcp in ((-15,0,0),(-30,0,0),(-45,0,0),(-30,15,0),(-45,25,0),(-30,15,10)):
        label=f'pip{pip}_dip{dip}_mcp{mcp}';sub=folder/label;sub.mkdir();d=restore(model,z);g=Guard(model);start=motor(d.ctrl);desired=d.ctrl.copy();desired[6]+=np.radians(pip);desired[7]+=np.radians(dip);desired[4]+=np.radians(mcp);target=motor(desired)
        if np.any(target<g.lo) or np.any(target>g.hi):
            row=dict(label=label,skipped='original_goal_bounds',whole_hand_supported_MF_reset_passed=False);rows.append(row);save_json(sub/'review.json',row);continue
        count=max(1,int(np.ceil(np.max(abs(target-start))/2)));previous=start.copy();commands=[];stop=[];samples=supported=0;contacts=set()
        for step in range(count+30):
            command=previous.copy();command[sid[active]]=np.rint(start[sid[active]]+(target[sid[active]]-start[sid[active]])*min(step+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9 and np.all(command>=g.lo) and np.all(command<=g.hi)
            d.ctrl[active]=ticks_to_sim(command)[active];assert np.array_equal(d.ctrl[:4],z['ctrl'][:4]) and np.array_equal(d.ctrl[8:],z['ctrl'][8:]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    load=sample_model(model,d,g.hand,cube);samples+=1;supported+=load['supported_sample'];contacts.update(load['loaded_hand_geoms'])
                if stop:break
            if stop:break
        passed=bool(not stop and samples==2250 and supported/2250>.8)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        points=np.zeros(6);tip_gap=float(mujoco.mj_geomDistance(model,d,cube,model.geom('mf_tip').id,.2,points))*1000
        row=dict(label=label,path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,samples=samples,whole_hand_support_fraction=supported/max(1,samples),loaded_hand_geoms=sorted(contacts),MF_tip_gap_mm=tip_gap,MF_PIP_upper_room_deg=float(np.degrees(g.qhi[6]-d.qpos[6])),cube_shift_mm=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000),cube_orientation_change_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0),yaw_change_deg=angle(d.geom_xmat[cube].reshape(3,3),B0,initial['up']),whole_hand_supported_MF_reset_passed=passed,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='MF_reposition_between_next_turn_pushes',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,sliding_friction_hypothesis=a.sliding_friction,rows=rows,other12_joint_commands_bitwise_unchanged=True,specific_MF_contact_not_required=True,whole_hand_support_includes_palm=True,original_guards_and_bounds_preserved=True,intermediate_reposition_not_final_hold_or_turn=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
