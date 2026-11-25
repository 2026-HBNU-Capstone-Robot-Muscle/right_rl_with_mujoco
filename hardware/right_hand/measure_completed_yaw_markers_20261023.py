"""Saved-image marker motion only. Does not certify robot registration."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'grasped_yaw_trial_20261023_234707_886202'
PHASES=('stage_4_fixed_grasp_observation','stage_5_grasped_small_yaw','stage_6_yaw_fixed_observation')


def main():
    r=json.loads((FOLDER/'result.json').read_bytes())
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    detector=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),hand_id=40,required_cube_count=1)
    rows=[]
    for f in frames:
        if f['phase'] not in PHASES or f['camera_read_monotonic']>r['rotation_evidence_cutoff_camera_monotonic']:continue
        a=cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=detector.detectMarkers(cv.cvtColor(a,cv.COLOR_BGR2GRAY))
        found={} if ids is None else {str(int(i)):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),corners)}
        row=dict(image=f['image'],phase=f['phase'],t=f['camera_read_monotonic']-r['enabled_at'],
            ids=list(found),corners={k:v.tolist() for k,v in found.items()})
        for ident,q in found.items():
            edge=q[1]-q[0]
            row['center_'+ident]=q.mean(axis=0).tolist()
            row['image_angle_'+ident]=float(np.degrees(np.arctan2(edge[1],edge[0])))
        rows.append(row)
    summaries={}
    for phase in PHASES:
        subset=[x for x in rows if x['phase']==phase]
        record=dict(frames=len(subset))
        for ident in ('5','40'):
            decoded=[x for x in subset if ident in x['ids']]
            record[ident]=dict(decoded=len(decoded))
            if decoded:
                record[ident].update(center_median=np.median([x['center_'+ident] for x in decoded],axis=0).tolist(),
                    angle_median=float(np.median([x['image_angle_'+ident] for x in decoded])),
                    angle_range=np.percentile([x['image_angle_'+ident] for x in decoded],[0,25,75,100]).tolist())
        summaries[phase]=record
    output=dict(source=FOLDER.name,phase_summary=summaries,observations=rows,
        relative_3D_rotation_verified=False,robot_registration_verified=False,
        warning='Image-plane angles alone are not physical yaw angles; translation and perspective matter.')
    (FOLDER/'saved_marker_motion.json').write_text(json.dumps(output,indent=2),encoding='utf-8')
    print(json.dumps(summaries,indent=2))
    print('CALIBRATION_REFERENCES')
    for p in ROOT.glob('*.py'):
        content=p.read_text(encoding='utf-8',errors='replace')
        if 'camera1_720.npz' in content:
            print(p.name,*[line.strip() for line in content.splitlines() if 'camera1_720.npz' in line],sep=' | ')


if __name__=='__main__':main()
