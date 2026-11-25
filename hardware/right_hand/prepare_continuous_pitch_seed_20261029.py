"""Preserve the prior conditional camera-derived pitch start as an experiment."""
import json,hashlib
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parent
source=ROOT/'right_hand_rotation_20261026/saved_marker_registration/conditional_raw_frame_start_pose.json'
r=json.loads(source.read_bytes())
v=next(v for v in r['candidates'] if v['consistent_with_known_start'])
R=np.asarray(v['R_cube_world']);rv=cv2.Rodrigues(R)[0].ravel();angle=np.linalg.norm(rv)
quat=np.r_[np.cos(angle/2),rv/angle*np.sin(angle/2)]
values=[*v['cube_center_world_m'],*quat.tolist()]
out=ROOT/'right_hand_rotation_20261029/pitch_then_slow_yaw_v1/camera_pitch_seed_hypothesis.json'
out.write_text(json.dumps(dict(seeds={'historical_camera_pitch_start':values},
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    physical_metric_registration_verified=False,candidate_not_authorization=True,devices_opened=False),indent=2)+'\n',encoding='utf-8')
print(str(out))
