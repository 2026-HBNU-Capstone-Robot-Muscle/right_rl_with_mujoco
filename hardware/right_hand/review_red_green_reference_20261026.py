"""Review the saved red/green reference without reopening devices."""
import hashlib,json
from pathlib import Path
import numpy as np
from center_marker_cube_pose_20261026 import estimate
ROOT=Path(__file__).resolve().parent
def main():
    pointer=ROOT/'leap_cube_red_green_result.json';digest=hashlib.sha256(pointer.read_bytes()).digest();folders=[p for p in ROOT.glob('cube_red_green_2026*') if (p/'result.json').is_file() and hashlib.sha256((p/'result.json').read_bytes()).digest()==digest]
    if len(folders)!=1:raise RuntimeError('Need one matching saved reference folder')
    folder=folders[0];source=json.loads(pointer.read_bytes());cal=np.load(ROOT/'camera1_720.npz');rows=[];observations=[]
    for profile in source.get('profiles',[]):
        for frame in profile.get('frames',[]):
            corners=frame['corners_px_by_id'];pose=estimate(corners,cal['K'],cal['dist']);rows.append(dict(image=frame['image'],profile=profile['name'],pose=pose));observations.append(dict(image=frame['image'],profile=profile['name'],corners=corners))
    profiles=[]
    for p in source.get('profiles',[]):
        group=[v for v in rows if v['profile']==p['name']];good=[v['pose'] for v in group if v['pose']['full_orientation_observable']];centers=np.asarray([v['cube_center_in_camera_m'] for v in good]);normals=np.asarray([v['face_normals_in_camera']['2'] for v in good]);mean=np.mean(normals,axis=0) if len(good) else None
        if mean is not None:mean/=np.linalg.norm(mean)
        profiles.append(dict(profile=p['name'],frames=len(group),all_three_ids=sum(all(str(i) in v['corners'] for i in (2,5,41)) for v in observations if v['profile']==p['name']),multi_face_candidates=len(good),reprojection_rms_px=[v['reprojection_rms_px'] for v in good],center_peak_to_peak_mm=(1000*np.ptp(centers,axis=0)).tolist() if len(good) else None,maximum_red_normal_deviation_deg=float(max(np.degrees(np.arccos(np.clip(normals@mean,-1,1))))) if len(good) else None))
    report=dict(status='saved_red_green_reference_review',folder=folder.name,source_sha256=digest.hex(),saved_frames=len(rows),profiles=profiles,frames=rows,camera_restoration=source.get('camera_restoration'),motors_accessed=source['motors_accessed'],motor_commands_sent=source['motor_commands_sent'],hardware_execution_allowed=False,real_devices_opened=False)
    (folder/'spin_invariant_pose_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');(folder/'all_face_detections.json').write_text(json.dumps(dict(frames=observations),indent=2)+'\n',encoding='utf-8');(ROOT/'right_hand_rotation_20261026/latest_red_reference_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2),flush=True)
if __name__=='__main__':main()
