"""Revisit nearby saved frames, avoiding another user capture."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from sac_registration_from_anchor import detect,marker_pose
from analyze_cube_visibility_pose_20261026 import pose
ROOT=Path(__file__).resolve().parent;RUN=ROOT/'loaded_repeat_20261026_170932_163649';OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    meta=json.loads((RUN/'camera/frames.json').read_bytes())['frames'];before=json.loads((RUN/'result.json').read_bytes())['placement_photo']['camera_read_monotonic'];window=[v for v in meta if before-12<=v['camera_read_monotonic']<=before+.25];rows=[];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist']
    for f in window[::2]:
        image,corners,ids,_=detect(RUN/'camera'/f['image'],dictionary)
        if 41 not in ids or 3 not in ids:continue
        R,t,error=marker_pose(corners,ids,41,.027,K,dist);planes=pose(corners[ids.index(3)],.0107,K,dist)
        rows.append(dict(image=f['image'],time_from_before_s=f['camera_read_monotonic']-before,phase=f['phase'],ids=ids,anchor_error_px=error,
            R_camera_anchor=R.tolist(),t_camera_anchor=t.tolist(),orange_poses=planes,corners={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)}))
    report=dict(status='saved_placement_neighbor_scan',frames_considered=len(window[::2]),rows=rows,source='loaded_repeat_20261026_170932_163649/camera/frames.json',hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'placement_neighbor_scan.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(considered=report['frames_considered'],found=len(rows),best=[{k:v for k,v in r.items() if k in ('image','time_from_before_s','phase','ids','anchor_error_px')} for r in sorted(rows,key=lambda v:v['anchor_error_px'])[:8]])),flush=True)
if __name__=='__main__':main()
