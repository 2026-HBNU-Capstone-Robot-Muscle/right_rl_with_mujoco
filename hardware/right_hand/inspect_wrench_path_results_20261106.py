"""Compact read-only summary of saved force-allocation path research."""
import argparse,json,numpy as np
from pathlib import Path
from hand_motor_mapping import SIM_TO_MOTOR_IDS,SIM_JOINT_NAMES
O=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip/quasistatic_wrench_path'
ap=argparse.ArgumentParser();ap.add_argument('--match',default='*');args=ap.parse_args()
for p in sorted(O.glob(args.match+'/review.json')):
    d=json.loads(p.read_bytes());v=d.get('recorded_goal_replay_verification')
    print(json.dumps(dict(path=str(p.relative_to(O)),task=d['task'],samples=d['samples'],phase=d['final_phase'],guard=d['guard_stop'],search_stop=d['search_stop'],yaw=d['final_yaw_deg'],shift=d['final_center_shift_mm'],fixed134=d.get('last134_goals_proved_constant',False),replay_safety=v['safety_passed'] if v else None,replay_common_grip=v['whole_grip_task_passed'] if v else None,replay_gap=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'] if v else None)))
    state=p.parent/'end_state.npz'
    if state.exists() and d['guard_stop']:
        z=np.load(state);errors=abs(z['qpos'][:16]-z['ctrl']);order=np.argsort(errors)[-3:][::-1];print(json.dumps(dict(case=p.parent.name,largest_joint_errors=[dict(joint=SIM_JOINT_NAMES[i],motor_id=SIM_TO_MOTOR_IDS[i],error_rad=float(errors[i]),goal_tick=int(round(z['ctrl'][i]*4096/(2*np.pi)+2048)),actual_tick=int(round(z['qpos'][i]*4096/(2*np.pi)+2048))) for i in order])))
