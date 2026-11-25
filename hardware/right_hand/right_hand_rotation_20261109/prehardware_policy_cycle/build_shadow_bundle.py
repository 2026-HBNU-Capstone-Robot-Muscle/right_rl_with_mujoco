"""Make a hardware-disabled numerical integration bundle from a model result."""
import json
import shutil
import numpy as np
from native_cycle import OUT,assets,Guard,sha,save_json
from cube_task_frame_20261105 import upright_yaw_target
from scipy.spatial.transform import Rotation


def main():
    source=OUT/'sac_current_contract_v4_20k/export_000000010256'
    review=json.loads((source/'evaluation_active_certified/review.json').read_text())
    if not review['rotation_and_hold_passed']:raise RuntimeError('Nominal certified model result required')
    folder=OUT/'shadow_bundle_nominal_only';folder.mkdir(exist_ok=True)
    m,z,ref,_,provenance=assets('active');guard=Guard(m)
    target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
    np.savez_compressed(folder/'reference.npz',start=ref['start'],goals=ref['goals'],
                        lower=guard.lo,upper=guard.hi,goal_quat=Rotation.from_matrix(target).as_quat()[[3,0,1,2]])
    for name,path in (('actor.npz',source/'actor.npz'),('shadow_runtime.py',OUT/'shadow_runtime.py')):
        shutil.copyfile(path,folder/name)
    save_json(folder/'bundle.json',{'format':'leap_offline_shadow_cycle_v1','residual_rad':.01,
              'control_seconds':.075,'maximum_goal_step_ticks':2,'sim_motor_ids':[8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1],
              'required_frame':'leap_sim_world','required_palm_reference':'policy_palm_position',
              'evidence':'nominal simulated rotation and hold only; perturbation cases not passed',
              'next_rotation_ready':False,'physical_camera_registration_verified':False,
              'hardware_execution_allowed':False,'model_provenance':provenance,
              'files_sha256':{name:sha(folder/name) for name in ('actor.npz','reference.npz','shadow_runtime.py')}})
    print(json.dumps({'bundle':str(folder),'hardware_execution_allowed':False,'devices_opened':0}),flush=True)


if __name__=='__main__':main()
