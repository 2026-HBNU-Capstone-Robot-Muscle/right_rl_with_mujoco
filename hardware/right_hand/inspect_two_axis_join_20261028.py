"""Saved geometry inputs and endpoints only; no hardware access."""
import json
from pathlib import Path
import numpy as np
from calibrated_quarter_plan_20261028 import LOADED_PATH

ROOT=Path(__file__).resolve().parent
def main():
    pitch=np.load(ROOT/'right_hand_rotation_20261028/two_axis_connection/saved_pitch/loaded_pitch_path.npz')
    yaw=np.load(LOADED_PATH)
    print('ENDPOINTS',json.dumps(dict(pitch_start=pitch['start'].tolist(),pitch_end=pitch['goals'][-1].tolist(),yaw_start=yaw['start'].tolist(),difference_ticks=(yaw['start']-pitch['goals'][-1]).tolist())))
    for name in ('right_hand_rotation_20261026/yaw_after_flip_camera_seed_v1/seed.json',
                 'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json'):
        p=ROOT/name
        if p.exists():
            r=json.loads(p.read_bytes())
            print('SEED',name,json.dumps({k:r[k] for k in ('actual_hand_ticks','commanded_hand_ticks','settled_cube_qpos','up_world','source_registration','initial_cube_pose_hypothesis') if k in r}))
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    rows=[v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3)]
    print('YAW_SEED',json.dumps(rows))

if __name__=='__main__':main()
