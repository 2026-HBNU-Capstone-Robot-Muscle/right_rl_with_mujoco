"""Compare actual parent and clear-grip joint states without device access."""
from pathlib import Path
import json
import hashlib
import numpy as np


def main():
    out=Path(__file__).resolve().parent;base=out/'next_cycle/handoff_refinement_20261110'
    pool_path=base/'early_CCD50_states/state_pool.npz';pool=np.load(pool_path,allow_pickle=False)
    target_path=base/'RF_clear_work_grip_joined_suffix/end_state.npz';target=dict(np.load(target_path,allow_pickle=False))
    ids=[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1];rows=[]
    for source_index in (1000,1300,1500,1700):
        index=int(np.flatnonzero(pool['indices']==source_index)[0]);q=pool['qpos'][index][:16];control=pool['ctrl'][index]
        rows.append(dict(source_index=source_index,actual_degrees=np.degrees(q).tolist(),command_degrees=np.degrees(control).tolist(),
            target_command_change_degrees=np.degrees(target['ctrl']-control).tolist(),cube_center=pool['qpos'][index][-7:-4].tolist()))
    result=dict(sim_motor_ids=ids,parent_pool_sha256=hashlib.sha256(pool_path.read_bytes()).hexdigest(),target_sha256=hashlib.sha256(target_path.read_bytes()).hexdigest(),
        target_actual_degrees=np.degrees(target['qpos'][:16]).tolist(),target_command_degrees=np.degrees(target['ctrl']).tolist(),rows=rows,devices_opened=0)
    canonical=np.load(out/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    snapshots=[]
    for name in ('first90_finish_checkpoint','next90_recenter_checkpoint'):
        path=base/name/'state.npz'
        if not path.exists():continue
        state=np.load(path,allow_pickle=False);C=state['qpos'][-7:-4]
        snapshots.append(dict(name=name,free_cube_body_center_m=C.tolist(),canonical_delta_mm=((C-canonical)*1000).tolist(),
            actual_joint_degrees=np.degrees(state['qpos'][:16]).tolist(),command_degrees=np.degrees(state['ctrl']).tolist()))
    result['executed_checkpoint_station_comparisons']=snapshots
    if len(snapshots)==2:result['checkpoint_center_separation_mm']=float(np.linalg.norm(np.array(snapshots[0]['free_cube_body_center_m'])-snapshots[1]['free_cube_body_center_m'])*1000)
    (base/'bridge_joint_reference_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(executed_checkpoint_station_comparisons=snapshots,checkpoint_center_separation_mm=result.get('checkpoint_center_separation_mm'))),flush=True)


if __name__=='__main__':main()
