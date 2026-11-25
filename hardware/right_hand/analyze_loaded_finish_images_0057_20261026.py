"""Provisional large-face image rotations, not a registered 3D measurement."""
import json
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'loaded_quarter_finish_trial_20261026_005744_776918';OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026'
# Load reusable definitions only; importing the old script would rewrite its
# historical output at import time. No old analysis/artifact is executed here.
source=(ROOT/'measure_loaded_green_grid_20261025.py').read_text(encoding='utf-8')
prefix=source.split("first,obj0,img0,debug0=",1)[0];namespace={'__file__':str(ROOT/'measure_loaded_green_grid_20261025.py')};exec(compile(prefix,'green_grid_definitions','exec'),namespace)
corners_and_grid=namespace['corners_and_grid'];K=namespace['K'];dist=namespace['dist'];det=namespace['det']
def solve(obj,img,intrinsic):
    _,rvs,tvs,_=cv.solvePnPGeneric(obj,img,intrinsic,dist,flags=cv.SOLVEPNP_IPPE);result=[]
    for rv,tv in zip(rvs,tvs):
        pr,_=cv.projectPoints(obj,rv,tv,intrinsic,dist);result.append(dict(error_px=float(np.linalg.norm(pr.reshape(-1,2)-img,axis=1).mean()),R=cv.Rodrigues(rv)[0].tolist(),t=tv.ravel().tolist()))
    return sorted(result,key=lambda a:a['error_px'])
def main():
    pairs={};frames=[]
    for name in ('cube_before_turn.jpg','cube_after_turn.jpg'):
        try:
            im,obj,img,_=corners_and_grid(SOURCE/name);pairs[name]=(obj,img);cs,ids,_=det.detectMarkers(im)
            for i,p in enumerate(img):cv.circle(im,tuple(np.rint(p).astype(int)),4,(255,0,255),2)
            cv.imencode('.png',im)[1].tofile(str(OUT/(name+'.grid.png')));frames.append(dict(image=name,grid_points=len(obj),markers=[] if ids is None else ids.ravel().tolist()))
        except Exception as exc:frames.append(dict(image=name,error=str(exc)))
    results=[]
    if len(pairs)==2:
        for scale in (.8,1.,1.2):
            intrinsic=K.copy();intrinsic[0,0]*=scale;intrinsic[1,1]*=scale;a=solve(*pairs['cube_before_turn.jpg'],intrinsic);b=solve(*pairs['cube_after_turn.jpg'],intrinsic)
            for i,v in enumerate(b):results.append(dict(focal_scale=scale,after_branch=i,before_error_px=a[0]['error_px'],after_error_px=v['error_px'],provisional_relative_camera_rotation_deg=float(np.linalg.norm(cv.Rodrigues(np.array(v['R'])@np.array(a[0]['R']).T)[0])*180/np.pi)))
    r=dict(frames=frames,results=results,hand_or_camera_3d_motion_not_compensated=True,calibration_identity_not_reverified=True,physical_quarter_turn_verified=False,user_observation='Tilted and stopped; user thinks thumb blocked the cube.')
    (OUT/'large_green_grid_review.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r),flush=True)
if __name__=='__main__':main()
