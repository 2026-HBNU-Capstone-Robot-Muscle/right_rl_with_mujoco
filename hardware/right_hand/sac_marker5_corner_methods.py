"""Offline comparison of subpixel/contour corners on the archived ID5 image."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from sac_registration_from_anchor import detect,cube_board
ROOT=Path(__file__).resolve().parent

def main():
    path=ROOT/'sequence_vision_20261013_213113_258536'/'placement_at_space.png'
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    raw,c,ids,_=detect(path,dictionary);gray=cv.cvtColor(raw,cv.COLOR_BGR2GRAY)
    base=np.array(c[ids.index(5)],np.float32).reshape(4,2)
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    dims=json.loads((ROOT/'cube_measurement_20261014.json').read_text())
    rotations=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_text())['face_rot'])
    pts,bids=cube_board.object_points(cube_mm=dims['cube_side_mm'],marker_mm=dims['id5_black_outer_side_mm_approximate'],layout='centers',face_rot=rotations)
    obj=np.array(pts[list(np.asarray(bids).ravel()).index(5)],np.float64)
    face=obj.mean(0)+(obj-obj.mean(0))*dims['cube_side_mm']/dims['id5_black_outer_side_mm_approximate']
    extent=np.array([[956,18],[951,263],[615,232],[652,0]],float)
    methods={'saved_contour':base}
    for win in (2,3,4,5):
        methods[f'subpix_saved_w{win}']=cv.cornerSubPix(gray,base.copy(),(win,win),(-1,-1),(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,50,.001))
    for label,ref in [('none',cv.aruco.CORNER_REFINE_NONE),('subpix',cv.aruco.CORNER_REFINE_SUBPIX),('apriltag',cv.aruco.CORNER_REFINE_APRILTAG)]:
        p=cv.aruco.DetectorParameters();p.cornerRefinementMethod=ref;p.adaptiveThreshWinSizeMax=53
        found,fi,_=cv.aruco.ArucoDetector(dictionary,p).detectMarkers(gray)
        if fi is not None and 5 in fi:methods[label]=np.array(found[list(fi.ravel()).index(5)]).reshape(4,2)
    rows=[]
    for name,corners in methods.items():
        ok,rv,tv=cv.solvePnP(obj,np.asarray(corners,dtype=float),K,dist)
        if not ok:continue
        pixels=cv.projectPoints(obj,rv,tv,K,dist)[0].reshape(4,2)
        fp=cv.projectPoints(face,rv,tv,K,dist)[0].reshape(4,2)
        rows.append(dict(method=name,corners_px=corners.tolist(),depth_mm=float(tv[2,0]*1000),
                         marker_reprojection_px=float(np.linalg.norm(pixels-corners,axis=-1).mean()),
                         manual_outer_extent_difference_px=float(np.linalg.norm(fp-extent,axis=-1).mean())))
    out=dict(status='corner_method_sensitivity_only',source=str(path),cases=rows,motor_commands_sent=0,
             caveat='Manual outer extent is approximate. Do not select a detector solely to match model height.')
    (ROOT/'sac_marker5_corner_methods_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in row.items() if k!='corners_px'} for row in rows],indent=2))

if __name__=='__main__':main()
