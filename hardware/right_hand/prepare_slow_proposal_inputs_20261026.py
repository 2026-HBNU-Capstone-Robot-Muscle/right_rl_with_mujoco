"""Compact geometry and image diagnostics from saved data only."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_run_0454'
RUN=ROOT/'slow_forward_placement_20261026_045454_900706'
def main():
    g=json.loads((OUT/'geometry_review.json').read_bytes())
    summary=[]
    for v in g['cases']:
        if v['motor_id'] in (3,):
            summary.append(dict(id=v['motor_id'],delta=v['delta_ticks'],thumb_displacement_mm=v['point_delta_mm']['th_tip'],range_ok=v['native_range_ok'],overlaps=v['overlaps_mm']))
    r=json.loads((RUN/'result.json').read_bytes());phases={}
    for p in dict.fromkeys(v['phase'] for v in r['samples']):
        s=[v for v in r['samples'] if v['phase']==p]
        phases[p]=dict(start=s[0]['t'],end=s[-1]['t'],duration=s[-1]['t']-s[0]['t'])
    namespace={'__file__':str(ROOT/'measure_loaded_green_grid_20261025.py')}
    text=(ROOT/'measure_loaded_green_grid_20261025.py').read_text(encoding='utf-8').split('first,obj0,img0,debug0=',1)[0]
    exec(compile(text,'grid_definitions','exec'),namespace)
    frames=[];pairs={}
    for name in ('cube_before_turn.jpg','cube_after_turn.jpg'):
        try:
            im,obj,img,dbg=namespace['corners_and_grid'](RUN/name)
            pairs[name]=(obj,img)
            _,ids,_=namespace['det'].detectMarkers(im)
            frames.append(dict(image=name,grid_points=len(obj),markers=[] if ids is None else ids.ravel().tolist()))
        except Exception as exc:frames.append(dict(image=name,error=str(exc)))
    angles=[]
    if len(pairs)==2:
        for scale in (.8,1.,1.2):
            k=namespace['K'].copy();k[0,0]*=scale;k[1,1]*=scale
            solutions=[]
            for name in ('cube_before_turn.jpg','cube_after_turn.jpg'):
                obj,img=pairs[name]
                _,rvs,tvs,_=cv.solvePnPGeneric(obj,img,k,namespace['dist'],flags=cv.SOLVEPNP_IPPE)
                ss=[]
                for rv,tv in zip(rvs,tvs):
                    pr,_=cv.projectPoints(obj,rv,tv,k,namespace['dist']);err=float(np.linalg.norm(pr.reshape(-1,2)-img,axis=1).mean())
                    ss.append((err,cv.Rodrigues(rv)[0]))
                solutions.append(sorted(ss,key=lambda a:a[0]))
            for j,b in enumerate(solutions[1]):
                angles.append(dict(focal_scale=scale,after_planar_branch=j,before_fit_px=solutions[0][0][0],after_fit_px=b[0],unregistered_camera_relative_change_deg=float(np.linalg.norm(cv.Rodrigues(b[1]@solutions[0][0][1].T)[0])*180/np.pi)))
    out=dict(thumb_yaw_probes=summary,phase_times=phases,image_diagnostic=dict(frames=frames,angles=angles,camera_or_hand_motion_not_compensated=True,physical_success_verified=False))
    (OUT/'proposal_inputs.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps(out,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
