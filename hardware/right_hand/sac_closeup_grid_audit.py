"""Offline solved-face grid diagnostic, independent of single-marker depth.

Uses eight green tile centers as a broad planar baseline. This assumes equal
tile pitch cube_side/3; it is not general scrambled-cube tracking or metrology.
"""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parent

def analyze(raw,marker,K,dist,side=55.):
    marker=np.asarray(marker,float).reshape(4,2);scale=np.mean(np.linalg.norm(marker-np.roll(marker,1,axis=0),axis=1))
    center=marker.mean(0);hsv=cv.cvtColor(raw,cv.COLOR_BGR2HSV)
    mask=cv.inRange(hsv,np.array([35,55,35]),np.array([100,255,255]))
    _,labels,stats,centers=cv.connectedComponentsWithStats(mask)
    selected=[]
    for stat,point in zip(stats[1:],centers[1:]):
        x,y,w,h,area=stat
        if .45*scale**2<area<6*scale**2 and .8*scale<np.linalg.norm(point-center)<3.2*scale:
            selected.append(point)
    if len(selected)!=8:raise ValueError(f'Expected eight surrounding green tiles, got {len(selected)}')
    # This diagnostic requires the face roughly image-upright; verify homography
    # residual and preserve the annotated assignment for inspection.
    pts=np.array(selected);pts=pts[np.argsort(pts[:,1])]
    pts=np.concatenate([row[np.argsort(row[:,0])] for row in (pts[:3],pts[3:5],pts[5:])])
    keep=[i for i in range(9) if i!=4]
    grid=np.array([[(i%3-1)*side/3,(i//3-1)*side/3] for i in keep],np.float32)
    und=cv.undistortPoints(pts[:,None].astype(float),K,dist,P=K).reshape(-1,2)
    H,_=cv.findHomography(grid,und,0)
    pred=cv.perspectiveTransform(grid[None],H)[0]
    undmarker=cv.undistortPoints(marker[:,None],K,dist,P=K).reshape(-1,2)
    onplane=cv.perspectiveTransform(undmarker[None],np.linalg.inv(H))[0]
    sides=np.linalg.norm(onplane-np.roll(onplane,1,axis=0),axis=1)
    obj=np.c_[grid,np.zeros(len(grid))].astype(float)
    ok,rv,tv=cv.solvePnP(obj,pts.astype(float),K,dist)
    if not ok:raise ValueError('Grid PnP failed')
    rigidpred=cv.projectPoints(obj,rv,tv,K,dist)[0].reshape(-1,2)
    return dict(grid_centers_px=pts.tolist(),homography_mean_error_px=float(np.linalg.norm(pred-und,axis=1).mean()),
                marker_side_from_grid_mm=sides.tolist(),marker_center_in_grid_mm=onplane.mean(0).tolist(),
                grid_pnp_mean_error_px=float(np.linalg.norm(rigidpred-pts,axis=1).mean()),
                grid_plane_center_camera_m=tv.ravel().tolist(),grid_rotation_camera=cv.Rodrigues(rv)[0].tolist()),pts

def main():
    capture=json.loads((ROOT/'leap_cube_closeup_result.json').read_text(encoding='utf-8'));folder=Path(capture['folder'])
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    rows=[]
    for i,frame in enumerate(capture['frames']):
        raw=cv.imdecode(np.fromfile(folder/frame['image'],np.uint8),cv.IMREAD_COLOR)
        row,pts=analyze(raw,frame['quality']['corners_px'],K,dist)
        rows.append(dict(image=frame['image'],**row))
        if i==0:
            for j,p in enumerate(pts):
                point=tuple(np.rint(p).astype(int));cv.circle(raw,point,5,(255,0,255),2)
                cv.putText(raw,str(j),point,cv.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
            (ROOT/'sac_closeup_grid_overlay.png').write_bytes(cv.imencode('.png',raw)[1].tobytes())
    sides=np.array([r['marker_side_from_grid_mm'] for r in rows])
    result=dict(status='offline_solved_green_grid_diagnostic',source=str(folder),motor_commands_sent=0,hardware_authorized=False,
                rows=rows,median_marker_sides_mm=np.median(sides,axis=0).tolist(),
                maximum_grid_homography_mean_error_px=max(r['homography_mean_error_px'] for r in rows),
                maximum_grid_pnp_mean_error_px=max(r['grid_pnp_mean_error_px'] for r in rows),
                limitations=['Assumes approximately equal 55/3 mm green tile-center pitch.',
                             'Sticker centroid, rounded edges, lens distortion and blur can bias this diagnostic.',
                             'No physical marker dimension or live runtime is changed.'])
    (ROOT/'sac_closeup_grid_audit_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))

if __name__=='__main__':main()
