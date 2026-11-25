"""Two planar-pose branches for a stored short rotation; not calibration."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_small_rotation_trial_20261022_151241_962114'


def main():
    r=json.loads((FOLDER/'marker_rotation_review.json').read_bytes())
    rows=[x for x in r['rows'] if x['phase']=='stage_4_small_rotation' and '5' in x['corners']]
    k=np.load(ROOT/'camera1_720.npz');K,D=k['K'],k['dist']
    half=.0107/2
    obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float64)
    frames=[]
    for row in rows:
        image=np.array(row['corners']['5'],np.float64)
        ok,rv,tv,errors=cv.solvePnPGeneric(obj,image,K,D,flags=cv.SOLVEPNP_IPPE_SQUARE)
        if not ok:continue
        poses=[dict(R=cv.Rodrigues(v)[0],t=t.ravel(),error=float(np.asarray(e).ravel()[0]))
               for v,t,e in zip(rv,tv,errors) if t.ravel()[2]>0]
        frames.append((row,poses))
    tracks=[]
    for branch in range(len(frames[0][1])):
        initial=frames[0][1][branch];previous=initial;track=[]
        for row,poses in frames:
            pose=min(poses,key=lambda p:np.linalg.norm(cv.Rodrigues(previous['R'].T@p['R'])[0]))
            rv=cv.Rodrigues(initial['R'].T@pose['R'])[0].ravel()*180/np.pi
            track.append(dict(image=row['image'],local_normal_twist_deg=float(rv[2]),
                              local_tilt_deg=float(np.linalg.norm(rv[:2])),reprojection_px=pose['error'],
                              center_camera_m=pose['t'].tolist(),R_camera=pose['R'].tolist()))
            previous=pose
        tracks.append(dict(initial_branch=branch,rows=track,
            last_three_median_twist_deg=float(np.median([x['local_normal_twist_deg'] for x in track[-3:]])),
            last_three_median_tilt_deg=float(np.median([x['local_tilt_deg'] for x in track[-3:]]))))
    out=dict(status='ambiguous_single_marker_relative_pose_diagnostic',tracks=tracks,
             physical_target_rotation_verified=False,world_gravity_yaw_not_established=True,
             marker_side_mm=10.7,cube_side_mm=55,
             limitation='Small foreshortened marker, glare, planar branches and local-normal axis. User reports no visible cube rotation.')
    (FOLDER/'relative_marker_pose_review.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps({**{k:v for k,v in out.items() if k!='tracks'},'tracks':[{k:v for k,v in t.items() if k!='rows'} for t in tracks]},indent=2))


if __name__=='__main__':main()
