"""Re-estimate old cube/hand poses using current dimensions, no devices."""
import json
from pathlib import Path
import cv2,numpy as np
from center_marker_cube_pose_20261026 import estimate
from sac_registration_from_anchor import marker_pose
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/original_measured_replay_v1'
def main():
    OUT.mkdir(exist_ok=True);cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);rows=[];model=json.loads((ROOT/'right_hand_rotation_20261026/saved_marker_registration/conditional_raw_frame_start_pose.json').read_bytes());mp=np.asarray(model['marker40_center_palm_m']);Rpm=np.asarray(model['R_palm_marker40'])
    for folder in ('sequence_vision_20261011_051349_782086','sequence_vision_20261011_051517_532925'):
        for path in sorted((ROOT/folder).glob('*.png')):
            image=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);det=MarkerDetector(dictionary,range(6),hand_id=40,required_cube_count=2);corners,ids,_=det.detectMarkers(gray);ids=[] if ids is None else ids.ravel().tolist();observed={str(i):np.asarray(c).reshape(4,2).tolist() for i,c in zip(ids,corners)};p=estimate(observed,K,dist)
            row=dict(folder=folder,image=path.name,ids=ids,pose=p)
            if p.get('full_orientation_observable') and 40 in ids:
                Rh,th,error=marker_pose(corners,ids,40,.0282,K,dist);C=Rh.T@(np.asarray(p['cube_center_in_camera_m'])-th);R=Rh.T@np.asarray(p['R_cube_camera']);up=np.array([0.,0.,1.]);nOrange=-R[:,0];nGreen=-R[:,1];heading=nOrange.copy();heading[2]=0;heading/=np.linalg.norm(heading);start=np.array([0.,-1.,0.]);yaw=float(np.degrees(np.arctan2(up@np.cross(start,heading),start@heading)));tilt=float(np.degrees(np.arccos(np.clip(nGreen@up,-1,1))));row.update(hand_marker_error_px=float(error),center_marker40_mm=(C*1000).tolist(),center_palm_mm=(mp+Rpm@C).tolist(),center_palm_units='m',R_cube_palm=(Rpm@R).tolist(),orange_heading_about_marker_normal_deg=yaw,green_up_error_to_marker_normal_deg=tilt)
                print(json.dumps(dict(folder=folder,image=path.name,yaw_deg=yaw,green_up_deg=tilt,center_palm_m=row['center_palm_mm'],cube_error_px=p['reprojection_rms_px'],hand_error_px=error)),flush=True)
            rows.append(row)
    report=dict(status='old_stage_images_current_dimension_pose_review',frames=rows,marker40_side_mm_assumed=28.2,cube_side_mm=55.,cube_marker_side_mm_assumed=10.7,hand_marker_plane_to_model_approximate=True,center_palm_field_contains_metres=True,corner_and_marker_mount_errors_remain=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'stage_cube_pose_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('SUMMARY',dict(photos=len(rows),with_hand_pose=sum('center_palm_mm' in r for r in rows)),flush=True)
if __name__=='__main__':main()
