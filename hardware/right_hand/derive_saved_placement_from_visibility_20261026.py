"""Conditional hand-relative start estimate; never authorizes robot motion."""
import hashlib,json,sys
from collections import Counter
from pathlib import Path
import cv2,numpy as np
from analyze_cube_visibility_pose_20261026 import pose
from center_marker_cube_pose_20261026 import FACE_BASES
from sac_registration_from_anchor import detect,marker_pose
from hand_registration_geometry import candidate
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration';VIS=ROOT/'cube_visibility_20261026_192140_408522'
def closest_rotation(matrix):
    U,_,V=np.linalg.svd(matrix);U[:,-1]*=np.linalg.det(U@V);return U@V
def main():
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];new=json.loads((VIS/'spin_invariant_pose_review.json').read_bytes());detections=json.loads((VIS/'all_face_detections.json').read_bytes())['frames'];table=[];spins=[]
    for row in new['frames']:
        p=row['pose']
        if row['profile'] not in ('original','manual_minus5','manual_minus6') or not p.get('full_orientation_observable'):continue
        f=next(v for v in detections if v['image']==row['image']);Ra,ta,e=marker_pose([np.asarray(f['corners']['41'])],[41],41,.027,K,dist)
        table.append(Ra.T@np.asarray(p['face_normals_in_camera']['5']))
        if '3' in p['marker_quarter_rotations']:spins.append(p['marker_quarter_rotations']['3'])
    up_anchor=np.mean(table,axis=0);up_anchor/=np.linalg.norm(up_anchor);spin=Counter(spins).most_common(1)[0][0]
    refs=json.loads((OUT/'same_run_pair_search.json').read_bytes())['pairs'];good=[v for v in refs if v['phase']!='stage_6_loaded_quarter_reference' and v['hand_error_px']<.8 and v['anchor_error_px']<.5]
    if not good:raise RuntimeError('No qualified same-run reference pairs')
    Rh=closest_rotation(np.mean([v['R_anchor_hand'] for v in good],axis=0));th=np.median([v['t_anchor_hand'] for v in good],axis=0)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);source=ROOT/('loaded_repeat_20261026_170932_163649/camera/frame_02304.jpg' if '--raw-frame' in sys.argv else 'loaded_repeat_20261026_170932_163649/cube_before_turn.jpg');image,corners,ids,_=detect(source,dictionary);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    anchor_quad=np.asarray(corners[ids.index(41)]).reshape(4,2);anchors=[]
    for window in (None,2,3,5):
        co=anchor_quad.copy().astype(np.float32)
        if window is not None:co=cv2.cornerSubPix(gray,co.reshape(-1,1,2),(window,window),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001)).reshape(4,2)
        Ra,ta,e=marker_pose([co],[41],41,.027,K,dist);anchors.append(dict(window=window,R=Ra,t=ta,error=e,corners=co))
    anchor=min(anchors,key=lambda v:v['error']);Ra,ta=anchor['R'],anchor['t'];planes=pose(corners[ids.index(3)],.0107,K,dist)
    n,u,v=(np.asarray(a,dtype=float) for a in FACE_BASES[3])
    for _ in range(spin):u,v=v,-u
    frame=np.column_stack([u,v,n]);registration=candidate();measure=registration['measurements'];model=json.loads((OUT/'model_coordinate_frames.json').read_bytes());Rph=np.diag([1.,-1.,-1.]);Rhm=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]]);Rpm=Rph@Rhm
    native=np.asarray(model['native_child_positions']['mf_bs']);legacy=np.asarray(model['legacy_child_positions']['mf_bs']);marker_p=native-Rph@np.array([measure['toward_fingertips'],measure['away_from_thumb'],measure['above_marker_plane']])*.001
    Rwp=np.asarray(model['palm_world_rotation']);twp=np.asarray(model['palm_world_position']);results=[]
    for p in planes:
        Rs=np.asarray(p['R']);center=np.asarray(p['t'])-Rs[:,2]*.0275;Rc=Rs@frame.T;cube_anchor=Ra.T@(center-ta);cm=Rh.T@(cube_anchor-th);cp=marker_p+Rpm@cm;Rp=Rpm@Rh.T@Ra.T@Rc
        green=Ra.T@Rc@np.array([0.,-1.,0.]);orange=Ra.T@Rc@np.array([-1.,0.,0.]);wrist=-Rh[:,1]
        err=float(np.degrees(np.arccos(np.clip(green@up_anchor,-1,1))));front=float(np.degrees(np.arccos(np.clip(orange@wrist,-1,1))))
        results.append(dict(reprojection_rms_px=p['reprojection_rms_px'],green_up_error_deg=err,orange_wrist_error_deg=front,consistent_with_known_start=bool(err<45 and front<45),
            center_in_marker40_mm=(cm*1000).tolist(),center_in_palm_mm=(cp*1000).tolist(),R_cube_palm=Rp.tolist(),cube_center_world_m=(twp+Rwp@cp).tolist(),R_cube_world=(Rwp@Rp).tolist()))
    up_p=Rpm@Rh.T@up_anchor;up_p/=np.linalg.norm(up_p);chosen=[v for v in results if v['consistent_with_known_start']]
    report=dict(status='conditional_saved_start_pose_from_current_visibility',source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        orange_marker_rotations=spins,orange_marker_quarter_rotation_mode=spin,same_run_reference_frames=[v['image'] for v in good],
        R_anchor_marker40=Rh.tolist(),t_anchor_marker40_m=th.tolist(),marker40_center_palm_m=marker_p.tolist(),R_palm_marker40=Rpm.tolist(),
        native_and_legacy_middle_root_local_positions_equal=bool(np.allclose(native,legacy)),legacy_world_frame_not_reused=True,
        marker40_assumed_palm_alignment_source='hand_registration_geometry.candidate; approximate and unverified',
        table_green_normal_in_anchor=up_anchor.tolist(),conditional_table_up_in_palm=up_p.tolist(),conditional_table_up_world=(Rwp@up_p).tolist(),
        anchor_refinement_selected_window=anchor['window'],anchor_reprojection_px=anchor['error'],anchor_refinement_errors=[dict(window=v['window'],error=v['error']) for v in anchors],
        candidates=results,one_prior_consistent_start_candidate=len(chosen)==1,hand_geometry_not_calibrated=True,
        current_table_surface_level_not_verified=True,centre_spins_assumed_unchanged_between_recordings=True,
        encoder_and_photo_time_match_not_verified=True,hardware_execution_allowed=False,real_devices_opened=False)
    name='conditional_raw_frame_start_pose.json' if '--raw-frame' in sys.argv else 'conditional_actual_start_pose.json'
    (OUT/name).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k in ('source','anchor_reprojection_px','one_prior_consistent_start_candidate') or k=='candidates'},indent=2),flush=True)
if __name__=='__main__':main()
