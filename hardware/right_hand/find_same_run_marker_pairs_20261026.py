"""Search saved images only for a current rigid ID40/41 relation."""
import json
from pathlib import Path
import cv2,numpy as np
from sac_registration_from_anchor import marker_pose
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'loaded_repeat_20261026_170932_163649/camera';OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
def main():
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR;p.adaptiveThreshWinSizeMax=53;det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),p)
    rows=json.loads((FOLDER/'frames.json').read_bytes())['frames'];found=[];checked=0
    for row in rows[::6]:
        path=FOLDER/row['image'];image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_GRAYSCALE);corners,ids,_=det.detectMarkers(image);checked+=1;decoded=[] if ids is None else ids.ravel().tolist()
        if 40 in decoded and 41 in decoded:
            Rh,th,eh=marker_pose(corners,decoded,40,.0282,K,dist);Ra,ta,ea=marker_pose(corners,decoded,41,.027,K,dist)
            found.append(dict(image=row['image'],phase=row['phase'],hand_error_px=eh,anchor_error_px=ea,R_anchor_hand=(Ra.T@Rh).tolist(),t_anchor_hand=(Ra.T@(th-ta)).tolist()))
            print('PAIR',json.dumps(found[-1]),flush=True)
        if checked%100==0:print('CHECKED',checked,'PAIRS',len(found),flush=True)
    (OUT/'same_run_pair_search.json').write_text(json.dumps(dict(images_checked=checked,sampling_stride=6,source_images=len(rows),pairs=found,real_devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(images_checked=checked,pairs=len(found)),flush=True)
if __name__=='__main__':main()
