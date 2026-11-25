"""Compare reached first-turn states with the initialized successful-next start."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,BOUND,sha,save_json,motor,SIM_TO_MOTOR_IDS
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'first_to_next_contact_bridge_20261111';folder.mkdir(exist_ok=True)
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    reference=base/'opposing_workspace_next90/start_state.npz';ref=dict(np.load(reference,allow_pickle=False));target=restore(model,ref);Cref=target.geom_xpos[cube].copy();Btarget=target.geom_xmat[cube].reshape(3,3).copy()
    first90=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90.)
    rows=[];sources={};pool_path=base/'early_CCD50_states/state_pool.npz';pool=np.load(pool_path,allow_pickle=False);sources[pool_path.relative_to(OUT).as_posix()]=sha(pool_path)
    entries=[]
    for i,index in enumerate(pool['indices']):
        z={k:pool[k][i].copy() for k in BOUND};z['time']=float(pool['time'][i]);entries.append((f'actual_policy_prefix_{int(index)}',z))
    for label,p in (('reached_first90',base/'initial_to_reached_workstation90/end_state.npz'),('successful_next90_start_initialized',reference)):
        entries.append((label,dict(np.load(p,allow_pickle=False))));sources[p.relative_to(OUT).as_posix()]=sha(p)
    for label,z in entries:
        d=restore(model,z);g=Guard(model);load=sample_model(model,d,g.hand,cube);forces={f:sum(v['normal_N'] for name,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(name).id])).name or '').startswith(f+'_')) for f in ('if','mf','rf','th')}
        delta=np.degrees(ref['ctrl']-d.ctrl)
        row=dict(label=label,time=float(d.time),center_world_m=d.geom_xpos[cube].tolist(),to_successful_next_start_mm=((Cref-d.geom_xpos[cube])*1000).tolist(),distance_to_next_start_mm=float(np.linalg.norm(Cref-d.geom_xpos[cube])*1000),orientation90_error_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),first90),orientation_to_next_start_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),Btarget),finger_normals_N=forces,whole_hand_support=load,native_guard=g.check(d),control_change_to_reference_deg={str(mid):float(delta[i]) for i,mid in enumerate(SIM_TO_MOTOR_IDS)})
        rows.append(row)
        print(json.dumps({k:row[k] for k in ('label','distance_to_next_start_mm','orientation90_error_deg','finger_normals_N','native_guard')}),flush=True)
    save_json(folder/'audit.json',dict(status='first_turn_to_successful_next_start_contract_audit',model_provenance=provenance,source_files_sha256=sources,reference=reference.relative_to(OUT).as_posix(),reference_sha256=sha(reference),rows=rows,
        successful_next_start_was_initialized_not_reached=True,actual_initial_state_to_second_turn_chain_not_passed=True,devices_opened=0,hardware_execution_allowed=False,new_SAC_training_steps=0))


if __name__=='__main__':main()
