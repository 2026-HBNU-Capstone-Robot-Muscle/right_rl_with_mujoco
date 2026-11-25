"""Correct-size image-only registration diagnostic, no devices or control."""
import hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration'
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'))
import cube_board
from hand_registration_geometry import candidate
from sac_registration_from_anchor import marker_pose
from marker_detector import MarkerDetector
def main():
    OUT.mkdir(parents=True,exist_ok=True);cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),cv2.aruco.DetectorParameters())
    def detect(path):
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);found={}
        # Independent same-frame contrast passes. Keep this diagnostic separate
        # from the runtime empty-hand marker gate; no historical corners reused.
        detectors=[det,MarkerDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),range(6),hand_id=41,required_cube_count=2)]
        for detector in detectors:
            corners,ids,_=detector.detectMarkers(gray)
            if ids is not None:
                for co,mid in zip(corners,ids.ravel()):found.setdefault(int(mid),co)
        if 'loaded_repeat_' in str(path) and not any(mid<6 for mid in found):
            enlarged=cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)
            for detector in detectors:
                corners,ids,_=detector.detectMarkers(enlarged)
                if ids is not None:
                    for co,mid in zip(corners,ids.ravel()):found.setdefault(int(mid),np.asarray(co)/2)
        return list(found.values()),list(found)
    pairs=[]
    for path in sorted(ROOT.glob('sequence_vision_*/anchor_empty_stage0.png')):
        corners,ids=detect(path)
        if 40 not in ids or 41 not in ids:continue
        Rh,th,eh=marker_pose(corners,ids,40,.0282,K,dist);Ra,ta,ea=marker_pose(corners,ids,41,.027,K,dist)
        pairs.append(dict(source=str(path.relative_to(ROOT)),R_anchor_hand=(Ra.T@Rh).tolist(),t_anchor_hand=(Ra.T@(th-ta)).tolist(),hand_error_px=eh,anchor_error_px=ea))
    print('REFERENCE_PAIRS',json.dumps(pairs),flush=True)
    reg=candidate();Rw=np.asarray(reg['hand_to_world_candidate']);gm=np.asarray(reg['grasp_site_in_marker_m_candidate']);face_rot=json.loads((ROOT/'cube_face_rot_result.json').read_bytes())['face_rot'];points,bids=cube_board.object_points(cube_mm=55,marker_mm=10.7,layout='centers',face_rot=face_rot)
    results=[]
    for name in ('cube_before_turn.jpg','cube_after_turn.jpg'):
        path=ROOT/'loaded_repeat_20261026_170932_163649'/name;corners,ids=detect(path);row=dict(image=str(path.relative_to(ROOT)),decoded_ids=ids)
        if 41 in ids:
            Ra,ta,err=marker_pose(corners,ids,41,.027,K,dist);row['anchor_error_px']=err;hyp=[]
            for mid,co in zip(ids,corners):
                if mid>=6:continue
                Rs,ts,es=marker_pose([co],[mid],mid,.0107,K,dist);center=ts-Rs[:,2]*.0275;anchor=Ra.T@(center-ta)
                # Marker normal and centre are invariant to a layer's in-plane
                # centre-marker rotation. Retain the planar ambiguity caveat.
                per=dict(marker_id=mid,marker_error_px=es,cube_center_in_anchor_mm=(anchor*1000).tolist(),normal_in_camera=Rs[:,2].tolist(),bridged_centers=[])
                for ref in pairs:
                    Rh=np.asarray(ref['R_anchor_hand']);th=np.asarray(ref['t_anchor_hand']);cm=Rh.T@(anchor-th);world=np.array([.11,0,.03])+Rw@(cm-gm)
                    per['bridged_centers'].append(dict(reference=ref['source'],center_in_marker40_mm=(cm*1000).tolist(),candidate_world_m=world.tolist()))
                hyp.append(per)
            row['single_face_center_candidates']=hyp
        results.append(row);print('IMAGE',json.dumps(row),flush=True)
    report=dict(status='saved_image_correct_size_registration_diagnostic',reference_pairs=pairs,images=results,
        marker_sizes_mm={'40':28.2,'41':27.,'cube_centers':10.7},cube_side_mm=55.,hand_geometry=reg,
        cross_session_rigidity_not_verified=True,small_marker_planar_pose_ambiguous=True,hand_origin_geometry_not_verified=True,
        hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
