import json
from pathlib import Path
import numpy as np
from sac_registration_from_anchor import marker_pose
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration';r=json.loads((OUT/'saved_red_marker_recovery.json').read_bytes());cal=np.load(ROOT/'camera1_720.npz');reg=json.loads((OUT/'conditional_raw_frame_start_pose.json').read_bytes());up=np.asarray(reg['table_green_normal_in_anchor'])
for f in r['frames_with_red_and_anchor'][:5]:
    for m in f['methods']:
        R,t,e=marker_pose([m['corners']['41']],[41],41,.027,cal['K'],cal['dist']);print(f['image'],m['correction_rate'],'bits',m['sampled_code_bit_errors'],m['sampled_border_bit_errors'],'red',round(m['red_context_fraction'],3),'anchor',round(e,3),'red planes',[(round(v['reprojection_rms_px'],3),round(float(np.degrees(np.arccos(np.clip((R.T@np.asarray(v['normal']))@up,-1,1)))),2)) for v in m['red_plane_candidates']])
