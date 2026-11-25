"""Independent full 55 mm face pose from saved photos; no device access.

Outer corners are approximate manual image coordinates, with declared 3 px
uncertainty. All planar pose branches are retained, not selected for success.
The old rigid hand/marker mounting transform remains provisional.
"""
import json,hashlib,sys
from pathlib import Path
import numpy as np
import cv2 as cv
from hand_registration_geometry import candidate
from register_original_sequence_images_20261024 import square

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_review'
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'))
import cube_board

def branches(obj,img,K,dist):
    answer=cv.solvePnPGeneric(np.asarray(obj,np.float64),np.asarray(img,np.float64),K,dist,flags=cv.SOLVEPNP_IPPE)
    results=[]
    for rv,tv in zip(answer[1],answer[2]):
        projected=cv.projectPoints(np.asarray(obj,np.float64),rv,tv,K,dist)[0].reshape(-1,2)
        results.append((cv.Rodrigues(rv)[0],tv.ravel(),float(np.linalg.norm(projected-np.asarray(img),axis=1).mean())))
    return results

def main():
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
    detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
    def read(path):
        data=path.read_bytes();im=cv.imdecode(np.frombuffer(data,np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
        return im,{int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),corners)},hashlib.sha256(data).hexdigest()
    points,ids=cube_board.object_points(cube_mm=55,marker_mm=10.7,layout='centers',
        face_rot=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_bytes())['face_rot']))
    marker=next(np.asarray(p) for p,i in zip(points,np.asarray(ids).ravel()) if int(i)==5)
    center=marker.mean(0);u=marker[1]-marker[0];u/=np.linalg.norm(u);v=marker[0]-marker[3];v/=np.linalg.norm(v)
    flat=square(.055)[:,:2];obj=center+flat[:,0,None]*u+flat[:,1,None]*v
    reg=candidate();Rw=np.array(reg['hand_to_world_candidate']);g=np.array(reg['grasp_site_in_marker_m_candidate'])
    up=np.array([3.9700269,.2684224,8.9676076]);up/=np.linalg.norm(up)
    registration=json.loads((OUT/'same_session_marker_registration.json').read_bytes());results=[]
    coords=[[[166,65],[376,60],[394,176],[146,180]],[[160,82],[376,80],[381,197],[126,199]]]
    for folder,rim in zip(registration['folders'],coords):
        name=folder['folder'];path=ROOT/name/'placement_at_space.png';im,seen,sha=read(path);rim=np.array(rim,float)
        emptyname=next(p['empty_reference'] for p in folder['registered'] if p['image']=='placement_at_space.png')
        _,empty,_=read(ROOT/name/emptyname)
        fits=[]
        for k in range(4):
            image_corners=np.roll(rim,-k,axis=0)
            h=cv.getPerspectiveTransform(flat.astype(np.float32),image_corners.astype(np.float32))
            xy=cv.perspectiveTransform(seen[5].astype(np.float64)[None],np.linalg.inv(h))[0]
            offset=xy.mean(0);scale=float(np.sum(flat*(xy-offset))/np.sum(flat**2))
            residual=float(np.sqrt(np.mean(np.sum((xy-(offset+scale*flat))**2,axis=1))))
            if scale>0:fits.append((residual,k,scale*.055,offset,image_corners))
        residual,k,black_side,offset,image_corners=min(fits,key=lambda x:x[0])
        cube=branches(obj,image_corners,K,dist);p40=branches(square(.0282),empty[40],K,dist)
        p41=branches(square(.027),empty[41],K,dist);p41placed=branches(square(.027),seen[41],K,dist)
        candidates=[]
        for ci,(Rc,tc,ce) in enumerate(cube):
            for hi,(Rh,th,he) in enumerate(p40):
                for ai,(Ra,ta,ae) in enumerate(p41):
                    for bi,(Rb,tb,be) in enumerate(p41placed):
                        local=Rh.T@(ta+Ra@(Rb.T@(tc-tb))-th)
                        R=Rw@Rh.T@Ra@Rb.T@Rc;world=np.array([.11,0,.03])+Rw@(local-g)
                        angle=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@up,-1,1))))
                        candidates.append(dict(branches=[ci,hi,ai,bi],center_world_m=world.tolist(),rotation_world=R.tolist(),
                            cube_rim_reprojection_px=ce,empty_hand_marker_px=he,empty_anchor_px=ae,placed_anchor_px=be,
                            initial_green_up_error_deg=angle,
                            image_fit_sum_px=ce+he+ae+be))
        candidates.sort(key=lambda x:x['image_fit_sum_px'])
        print('WHOLE_FACE',name,'black_marker_estimate_mm',round(black_side*1000,2),
            'best_image_fit_center',np.round(candidates[0]['center_world_m'],4).tolist(),
            'green_up_error',round(candidates[0]['initial_green_up_error_deg'],2),flush=True)
        results.append(dict(folder=name,source_image_sha256=sha,manual_outer_corners_px=rim.tolist(),
            manual_outer_corner_uncertainty_px=3,source_cube_side_mm=55,
            marker_black_side_inferred_from_face_ratio_mm=black_side*1000,
            marker_offset_in_face_plane_mm=(offset*1000).tolist(),marker_square_fit_residual_mm=residual*1000,
            empty_reference=emptyname,all_planar_pose_combinations=candidates))
    (OUT/'whole_face_rim_registration.json').write_text(json.dumps(dict(captures=results,
        marker_size_not_used_for_cube_depth=True,hand_mount_transform=reg,
        geometry_calibration_verified=False,pose_branch_not_selected_for_simulated_success=True,
        raw_images_unchanged=True,hardware_commands_sent=0),indent=2),encoding='utf-8')

if __name__=='__main__':main()
