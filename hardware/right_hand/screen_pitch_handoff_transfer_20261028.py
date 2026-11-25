"""Transfer retention sensitivity, not certification of a ready yaw grasp."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from types import SimpleNamespace
from mujoco_playground import registry
import replay_pitch_yaw_join_20261028 as replay
from pitch_handoff_plan_20261028 import OUT,TRANSFER_PATH,TRANSFER_SEED
from book_empty_preparation_plan import FRONTS

def main():
    folder=OUT/'transfer_retention';folder.mkdir(parents=True,exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);cases=[]
    for front in FRONTS:
        replay.OUT=folder/f'front_{front:g}'
        replay.ROUTES=OUT
        r=replay.run(SimpleNamespace(route='transfer_path',nominal_only=False,append_yaw=False,seat_cycle=False),env,front=front)
        for row in r['cases']:
            v=dict(row)
            v['accepted_for_transfer_observation']=bool(
                row['stop'] is None and row['acceptance_checks']['motion_health_passed']
                and row['acceptance_checks']['final_hand_contact'] and row['red_up_deg']<15
                and not row['external_cube_contact'] and row['max_self_mm']<=1e-6
                and row['max_support_mm']<=.15 and row['max_cube_mm']<=1
                and row['max_tracking_rad']<=.18 and row['native_range_excess_rad']<=1e-6
                and row['max_no_hand_contact_seconds']<=.25)
            cases.append(v)
        result=dict(status='conditional_transfer_retention_only',cases=cases,
                    all_retention_cases_passed=all(v['accepted_for_transfer_observation'] for v in cases),
                    desired_yaw_start_seating_not_verified=True,subsequent_yaw_not_allowed=True,
                    initial_cube_pose_is_uncertified_hypothesis=True,
                    transfer_path_sha256=hashlib.sha256(TRANSFER_PATH.read_bytes()).hexdigest(),
                    initial_cube_seed_sha256=hashlib.sha256(TRANSFER_SEED.read_bytes()).hexdigest(),
                    hardware_execution_allowed=False,devices_opened=False)
        (folder/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        print('RETENTION',front,sum(v['accepted_for_transfer_observation'] for v in cases),'/',len(cases),flush=True)

if __name__=='__main__':main()
