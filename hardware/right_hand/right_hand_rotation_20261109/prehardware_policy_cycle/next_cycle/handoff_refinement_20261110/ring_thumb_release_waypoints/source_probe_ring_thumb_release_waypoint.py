"""Explicit pressure-release waypoints from an executed turn snapshot.

All bodies stay dynamic. Only RF/TH commands change. A supported intermediate
waypoint is not a completed turn or a final upper-clearance certificate.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import orientation_error_deg


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'ring_thumb_release_waypoints'
    if folder.exists():raise RuntimeError('Preserve earlier release probes')
    source=base/'ring_thumb_turn_checkpoint/state.npz';z=dict(np.load(source,allow_pickle=False));model,initial,_,_,provenance=assets('active_ccd50');data=restore(model,z);guard=Guard(model);cube=model.geom('cube').id
    C0=data.geom_xpos[cube].copy();B0=data.geom_xmat[cube].reshape(3,3).copy();closing=np.zeros(16);sid=np.asarray(SIM_TO_MOTOR_IDS)-1;active=np.arange(8,16)
    for finger,indices in (('rf',np.arange(8,12)),('th',np.arange(12,16))):
        options=[c for c in data.contact if cube in (int(c.geom1),int(c.geom2)) and (model.body(int(model.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)])).name or '').startswith(finger+'_')]
        if not options:raise RuntimeError('Expected loaded side is absent: '+finger)
        c=min(options,key=lambda c:c.dist);other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);J=np.zeros((3,model.nv));mujoco.mj_jac(model,data,J,None,c.pos,int(model.geom_bodyid[other]))
        closing[indices]=(-J[:,:16].T@n/model.actuator_gainprm[:,0])[indices]
    folder.mkdir();(folder/'source_probe_ring_thumb_release_waypoint.py').write_bytes(Path(__file__).read_bytes());rows=[]
    for release,retreat in ((.8,0.),(1.6,0.),(0.,4.),(.8,4.)):
        desired=z['ctrl']-closing*release;desired[9]-=np.radians(retreat);target=motor(desired)
        label=f'release{release:g}_retreat{retreat:g}';sub=folder/label;sub.mkdir()
        if np.any(target<guard.lo) or np.any(target>guard.hi):
            row=dict(label=label,skipped='native_command_bounds',intermediate_supported=False);rows.append(row);save_json(sub/'review.json',row);continue
        actual=restore(model,z);check=Guard(model);start=motor(actual.ctrl);previous=start.copy();commands=[];stop=[];supported=samples=0
        count=max(1,int(np.ceil(np.max(abs(target-start))/2)))
        for step in range(count+30):
            goal=previous.copy();goal[sid[active]]=np.rint(start[sid[active]]+(target[sid[active]]-start[sid[active]])*min(step+1,count)/count)
            assert np.max(abs(goal-previous))<=2+1e-9 and np.all(goal>=check.lo) and np.all(goal<=check.hi)
            actual.ctrl[active]=ticks_to_sim(goal)[active];previous=goal.copy();commands.append(goal.copy());assert np.array_equal(actual.ctrl[:8],z['ctrl'][:8])
            for _ in range(75):
                mujoco.mj_step(model,actual);stop=check.check(actual)
                if np.linalg.norm(actual.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:samples+=1;supported+=sample_model(model,actual,check.hand,cube)['supported_sample']
                if stop:break
            if stop:break
        load=sample_model(model,actual,check.hand,cube);forces={finger:sum(v['normal_N'] for geom,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(geom).id])).name or '').startswith(finger+'_')) for finger in ('rf','th')}
        shift=float(np.linalg.norm(actual.geom_xpos[cube]-C0)*1000);angle=orientation_error_deg(actual.geom_xmat[cube].reshape(3,3),B0);room=float(np.degrees(check.qhi[9]-actual.qpos[9]))
        passed=bool(not stop and samples==2250 and supported/2250>.8 and shift<2 and angle<3)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands)
        np.savez_compressed(sub/'end_state.npz',**{k:getattr(actual,k) for k in BOUND},time=actual.time,up=initial['up'],initial_cube_rotation=initial['initial_cube_rotation'],initial_cube_center=initial['initial_cube_center'])
        row=dict(label=label,release_effort_hypothesis_N=release,RF14_control_retreat_deg=retreat,steps=len(commands),stop=stop,intermediate_supported=passed,
            samples=samples,whole_hand_support_fraction=supported/max(1,samples),cube_shift_mm=shift,orientation_change_deg=angle,RF14_upper_room_deg=room,final_finger_normals_N=forces,worst=check.worst,path=sub.relative_to(OUT).as_posix())
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps(row),flush=True)
    viable=[r for r in rows if r['intermediate_supported']];selected=min(viable,key=lambda r:max(r['final_finger_normals_N'].values())-min(r['RF14_upper_room_deg'],8)*.1) if viable else None
    save_json(folder/'review.json',dict(status='explicit_ring_thumb_release_waypoint_probes',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),model_provenance=provenance,rows=rows,selected_path=selected['path'] if selected else None,
        poses_executed_without_cube_reset=True,other_IF_MF_commands_unchanged=True,intermediate_waypoint_is_not_final_hold_or_turn=True,force_labels_not_actual450mA_conversion=True,
        original_guards_and_bounds_preserved=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
