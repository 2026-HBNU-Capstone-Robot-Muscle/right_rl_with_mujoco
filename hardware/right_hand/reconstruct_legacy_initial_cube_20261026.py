"""Conditional original P0 cube pose from same-run rigid wrist reference."""
import json
from pathlib import Path
import cv2,numpy as np
from center_marker_cube_pose_20261026 import FACE_BASES
from analyze_cube_visibility_pose_20261026 import pose
from sac_registration_from_anchor import marker_pose
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/original_measured_replay_v1'
def main():
    Kdist=np.load(ROOT/'camera1_720.npz');K,dist=Kdist['K'],Kdist['dist'];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);registration=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/conditional_raw_frame_start_pose.json').read_bytes());model=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/model_coordinate_frames.json').read_bytes());Rpm=np.asarray(registration['R_palm_marker40']);mp=np.asarray(registration['marker40_center_palm_m']);Rwp=np.asarray(model['palm_world_rotation']);wp=np.asarray(model['palm_world_position']);cases=[]
    def detect(path):
        im=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.cornerRefinementWinSize=3;p.adaptiveThreshWinSizeMax=53;co,ids,_=cv2.aruco.ArucoDetector(dictionary,p).detectMarkers(gray);ids=[] if ids is None else ids.ravel().tolist();return co,ids
    for run in ('sequence_vision_20261011_051349_782086','sequence_vision_20261011_051517_532925'):
        folder=ROOT/run;refs=[]
        for name in ('preflight_not_ready.png','configure_off_not_ready.png','enable_hold_not_ready.png','stage_0_end.png'):
            path=folder/name
            if not path.is_file():continue
            co,ids=detect(path)
            if 40 not in ids or 41 not in ids:continue
            Rh,th,he=marker_pose(co,ids,40,.0282,K,dist);Ra,ta,ae=marker_pose(co,ids,41,.027,K,dist);refs.append(dict(image=name,R_anchor_hand=(Ra.T@Rh).tolist(),t_anchor_hand=(Ra.T@(th-ta)).tolist(),hand_error=he,anchor_error=ae))
        co,ids=detect(folder/'placement_at_space.png');row=dict(run=run,reference_candidates=refs,placed_ids=ids,candidates=[])
        if refs and 41 in ids and 5 in ids:
            ref=min(refs,key=lambda v:v['hand_error']+v['anchor_error']);Rh=np.asarray(ref['R_anchor_hand']);th=np.asarray(ref['t_anchor_hand']);Ra,ta,ae=marker_pose(co,ids,41,.027,K,dist)
            for p in pose(co[ids.index(5)],.0107,K,dist):
                Rs=np.asarray(p['R']);center=np.asarray(p['t'])-Rs[:,2]*.0275;anchor=Ra.T@(center-ta);C=Rh.T@(anchor-th);cp=mp+Rpm@C
                for k in range(4):
                    n,u,v=(np.asarray(q,dtype=float) for q in FACE_BASES[5])
                    for _ in range(k):u,v=v,-u
                    Rc=Rs@np.column_stack([u,v,n]).T;Rp=Rpm@Rh.T@Ra.T@Rc;green=-Rp[:,1];orange=-Rp[:,0];up=np.array([0,0,-1]);wrist=np.array([-1,0,0]);ge=float(np.degrees(np.arccos(np.clip(green@up,-1,1))));oe=float(np.degrees(np.arccos(np.clip(orange@wrist,-1,1))))
                    if ge<30 and oe<35:row['candidates'].append(dict(source_reference=ref['image'],anchor_error_px=ae,green_up_error_deg=ge,orange_wrist_error_deg=oe,green_marker_quarter=k,center_palm_m=cp.tolist(),R_cube_palm=Rp.tolist(),center_world_m=(wp+Rwp@cp).tolist(),R_cube_world=(Rwp@Rp).tolist(),marker_error_px=p['reprojection_rms_px']))
        cases.append(row);print(json.dumps(row),flush=True)
    result=dict(status='original_loaded_P0_camera_pose_hypotheses',cases=cases,user_green_marker_size_mm_approx=10.7,marker41_size_mm_approx=27.,known_green_up_orange_wrist_used_as_prior=True,hand_registration_unverified=True,hardware_execution_allowed=False,real_devices_opened=False);(OUT/'initial_cube_camera_hypotheses.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
