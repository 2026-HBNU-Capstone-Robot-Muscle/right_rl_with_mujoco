"""Assemble a push-contact planning reference, explicitly not an actual state."""
from pathlib import Path
import numpy as np
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';folder=base/'opposing_transport_waypoint'
    if folder.exists():raise RuntimeError('Preserve previous waypoint')
    source=base/'first90_RF_shape_before_adduction/RF_park_MF_then_RF_shape_then_adduct/phase0_end_state.npz'
    middle=base/'opposing_actual_cube_contact_geometry/opposed_X_mf_proposal.npz'
    final=base/'opposing_workspace_Z_seed122/free_hold_tests/force0p8/end_state.npz'
    z=dict(np.load(source,allow_pickle=False));mf=dict(np.load(middle,allow_pickle=False));target=dict(np.load(final,allow_pickle=False))
    model,_,_,_,provenance=assets('active_ccd50');data=restore(model,z);data.qpos[4:8]=mf['qpos'][4:8];data.ctrl[4:8]=mf['ctrl'][4:8]
    data.qpos[12:16]=target['qpos'][12:16];data.ctrl[12:16]=target['ctrl'][12:16]
    import mujoco
    mujoco.mj_forward(model,data);guard=Guard(model);reasons=guard.check(data)
    if reasons:raise RuntimeError('Assembled waypoint geometry is invalid: '+str(reasons))
    values={key:getattr(data,key).copy() for key in BOUND};values['qpos'][-7:-4]=target['qpos'][-7:-4]
    folder.mkdir();np.savez_compressed(folder/'goal_reference_only.npz',**values,time=data.time)
    save_json(folder/'manifest.json',dict(status='assembled_push_reference_not_an_executed_state',source_files_sha256={p.relative_to(OUT).as_posix():sha(p) for p in (source,middle,final)},
        reference=(folder/'goal_reference_only.npz').relative_to(OUT).as_posix(),reference_sha256=sha(folder/'goal_reference_only.npz'),model_provenance=provenance,
        checked_joint_combination_at_actual_cube_not_at_hypothetical_destination=True,checked_geometry=guard.last,
        middle_targets_current_cube_plus_X_face=True,desired_station_from_separate_initialized_opposed_grip=True,
        actual_start_state_must_be_source_not_this_reference=True,no_arrival_or_hold_claim=True,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))
    print('push_reference_prepared',flush=True)


if __name__=='__main__':main()
