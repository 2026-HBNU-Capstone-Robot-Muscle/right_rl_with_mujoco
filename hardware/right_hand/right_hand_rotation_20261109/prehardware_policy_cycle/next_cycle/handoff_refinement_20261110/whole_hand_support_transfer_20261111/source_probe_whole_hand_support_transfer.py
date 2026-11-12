"""Let the index base carry the cube while all other joint commands stay fixed.

Temporary tilt and finger contact changes are allowed. Original native guards,
joint bounds and2tick/75ms remain unchanged; no final-turn success is inferred.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'whole_hand_support_transfer_20261111'
    if folder.exists():raise RuntimeError('Preserve previous support-transfer study')
    source=base/'reached_first90_reverse_quarter_probe/first90_space_hold_state.npz';refpath=base/'opposing_workspace_next90/start_state.npz'
    model,initial,_,_,provenance=assets('active_ccd50');z=dict(np.load(source,allow_pickle=False));reference=np.load(refpath,allow_pickle=False);origin=restore(model,z);cube=model.geom('cube').id;sid=np.asarray(SIM_TO_MOTOR_IDS)-1;motor_id=int(sid[0]);C0=origin.geom_xpos[cube].copy();targetC=reference['qpos'][-7:-4];targetB=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.)
    folder.mkdir();(folder/'source_probe_whole_hand_support_transfer.py').write_bytes(Path(__file__).read_bytes());rows=[]
    J=np.zeros((3,model.nv));mujoco.mj_jac(model,origin,J,None,C0,model.body('if_bs').id);derivative=J[:,0]*np.pi/180*1000
    for delta in (-18.,-12.,-6.,6.,12.,18.):
        d=restore(model,z);g=Guard(model);start=motor(d.ctrl);goal=start.copy();goal[motor_id]=round(start[motor_id]+delta*4096/360);label=f'ID8_delta{delta:+g}';sub=folder/label;sub.mkdir()
        if not np.all((goal>=g.lo)&(goal<=g.hi)):
            row=dict(label=label,skipped='original_goal_bounds',intermediate_whole_hand_supported=False);rows.append(row);save_json(sub/'review.json',row);continue
        count=max(1,int(np.ceil(abs(goal[motor_id]-start[motor_id])/2)));commands=[];previous=start.copy();stop=[];samples=supported=0;contact_geoms=set()
        for step in range(count+30):
            command=previous.copy();command[motor_id]=round(start[motor_id]+(goal[motor_id]-start[motor_id])*min(step+1,count)/count)
            assert np.max(abs(command-previous))<=2+1e-9;d.ctrl[0]=ticks_to_sim(command)[0];assert np.array_equal(d.ctrl[1:],z['ctrl'][1:]);commands.append(command.copy());previous=command.copy()
            for _ in range(75):
                mujoco.mj_step(model,d);stop=g.check(d)
                if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                if step>=count:
                    load=sample_model(model,d,g.hand,cube);samples+=1;supported+=load['supported_sample'];contact_geoms.update(load['loaded_hand_geoms'])
                if stop:break
            if stop:break
        load=sample_model(model,d,g.hand,cube);normals={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_')) for f in ('if','mf','rf','th')}
        finalC=d.geom_xpos[cube].copy();passed=bool(not stop and samples==2250 and supported/2250>.8)
        np.savez_compressed(sub/'research_goals.npz',start=start,goals=commands);np.savez_compressed(sub/'end_state.npz',**{k:getattr(d,k) for k in BOUND},time=d.time,up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
        row=dict(label=label,path=sub.relative_to(OUT).as_posix(),steps=len(commands),stop=stop,requested_root_change_deg=delta,samples=samples,whole_hand_support_fraction=supported/max(1,samples),loaded_hand_geoms_in_hold=sorted(contact_geoms),final_finger_normals_N=normals,
            center_world_m=finalC.tolist(),center_change_mm=((finalC-C0)*1000).tolist(),distance_to_successful_next_start_mm=float(np.linalg.norm(finalC-targetC)*1000),global90_orientation_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),targetB),intermediate_whole_hand_supported=passed,worst=g.worst,terminal_guard=g.last)
        rows.append(row);save_json(sub/'review.json',row);print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(folder/'review.json',dict(status='whole_hand_supported_index_base_transfer_probe',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),goal_reference=refpath.relative_to(OUT).as_posix(),goal_reference_sha256=sha(refpath),model_provenance=provenance,index_support_point_jacobian_mm_per_deg=derivative.tolist(),rows=rows,
        other15_joint_commands_bitwise_unchanged=True,finger_contacts_may_change=True,original_guards_and_bounds_preserved=True,intermediate_tilt_not_final_pose_hold=True,next_rotation_and_hardware_not_certified=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
