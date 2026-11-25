"""Evaluate real saved images and centre-spin invariance without devices."""
import hashlib,json
from pathlib import Path
import numpy as np
from center_marker_cube_pose_20261026 import estimate
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'cube_visibility_20261026_192140_408522'
def main():
    data=json.loads((FOLDER/'all_face_detections.json').read_bytes());cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];rows=[]
    for frame in data['frames']:
        p=estimate(frame['corners'],K,dist);rows.append(dict(image=frame['image'],profile=frame['profile'],pose=p))
    summary=[]
    for name in ('original','manual_minus5','manual_minus6','manual_minus7'):
        group=[v for v in rows if v['profile']==name];good=[v['pose'] for v in group if v['pose']['full_orientation_observable']]
        centers=np.asarray([v['cube_center_in_camera_m'] for v in good]);normals=np.asarray([v['face_normals_in_camera']['5'] for v in good])
        norm=np.mean(normals,axis=0) if len(normals) else None
        if norm is not None:norm/=np.linalg.norm(norm)
        summary.append(dict(profile=name,frames=len(group),multi_face_candidates=len(good),rms_px=[v['reprojection_rms_px'] for v in good],
            centre_peak_to_peak_mm=(1000*np.ptp(centers,axis=0)).tolist() if len(centers) else None,
            maximum_normal_difference_deg=float(max(np.degrees(np.arccos(np.clip(normals@norm,-1,1))))) if len(normals) else None))
    invariance=[]
    source=next((v for v in rows if v['pose']['full_orientation_observable']),None)
    if source is not None:
        frame=next(v for v in data['frames'] if v['image']==source['image']);base=source['pose'];ids=base['visible_face_ids']
        for face in ids:
            for turns in (1,2,3):
                changed={key:np.asarray(value).copy() for key,value in frame['corners'].items()};changed[str(face)]=np.roll(changed[str(face)],turns,axis=0);p=estimate(changed,K,dist)
                angle=float(np.degrees(np.arccos(np.clip((np.trace(np.asarray(base['R_cube_camera']).T@np.asarray(p['R_cube_camera']))-1)/2,-1,1))))
                displacement=float(1000*np.linalg.norm(np.asarray(base['cube_center_in_camera_m'])-np.asarray(p['cube_center_in_camera_m'])))
                accepted=bool(p['full_orientation_observable'] and angle<.001 and displacement<.001)
                invariance.append(dict(face=face,centre_marker_quarter_turns=turns,whole_cube_rotation_error_deg=angle,center_error_mm=displacement,passed=accepted))
        single=estimate({str(ids[0]):frame['corners'][str(ids[0])]},K,dist)
        assert single['full_orientation_observable'] is False and single['whole_cube_yaw_not_observable'] is True
        assert all(v['passed'] for v in invariance)
    report=dict(status='spin_invariant_saved_image_pose_review',source_sha256=hashlib.sha256((FOLDER/'all_face_detections.json').read_bytes()).hexdigest(),
        software_sha256=hashlib.sha256((ROOT/'center_marker_cube_pose_20261026.py').read_bytes()).hexdigest(),profiles=summary,frames=rows,
        centre_spin_invariance_checks=invariance,single_face_yaw_is_not_certified=True,
        actual_layer_turn_not_recorded=True,synthetic_corner_correspondence_rotation_is_a_geometry_regression=True,
        real_devices_opened=False,hardware_execution_allowed=False)
    (FOLDER/'spin_invariant_pose_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(profiles=summary,invariance=invariance),indent=2),flush=True)
if __name__=='__main__':main()
