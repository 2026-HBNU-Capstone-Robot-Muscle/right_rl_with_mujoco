"""Same-session historical image registration, corrected dimensions, no devices.

Square marker poses remain provisional. Historical photos are never modified.
Only a rigid camera/marker transform is estimated; no joint zeros are fitted.
"""
import hashlib
import json
import sys
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_review'
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'))
import cube_board
from hand_registration_geometry import candidate


def pose(obj,img,K,dist):
    ok,rv,tv=cv.solvePnP(np.asarray(obj,np.float64),np.asarray(img,np.float64),K,dist)
    if not ok:raise ValueError('PnP failed')
    projected,_=cv.projectPoints(np.asarray(obj,np.float64),rv,tv,K,dist)
    err=float(np.linalg.norm(projected.reshape(-1,2)-np.asarray(img).reshape(-1,2),axis=1).mean())
    return cv.Rodrigues(rv)[0],tv.ravel(),err


def square(side):
    s=side/2
    return np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    with np.load(ROOT/'camera1_720.npz') as c:K,dist=c['K'],c['dist']
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector=cv.aruco.ArucoDetector(dictionary)
    face_rot=tuple(json.loads((ROOT/'cube_face_rot_result.json').read_bytes())['face_rot'])
    points,ids=cube_board.object_points(cube_mm=55.,marker_mm=10.7,layout='centers',face_rot=face_rot)
    pointmap={int(i):np.asarray(p) for i,p in zip(np.asarray(ids).ravel(),points)}
    reg=candidate();Rw=np.array(reg['hand_to_world_candidate']);g=np.array(reg['grasp_site_in_marker_m_candidate'])
    folders=[]
    for foldername in ('sequence_vision_20261011_051349_782086','sequence_vision_20261011_051517_532925'):
        folder=ROOT/foldername;frames=[]
        for p in sorted(folder.glob('*.png')):
            frame=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR)
            corners,decoded,_=detector.detectMarkers(cv.cvtColor(frame,cv.COLOR_BGR2GRAY))
            decoded=[] if decoded is None else list(map(int,decoded.ravel()))
            seen={i:c.reshape(4,2) for i,c in zip(decoded,corners)}
            row=dict(image=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),ids=decoded)
            for marker,side in ((40,.0282),(41,.027)):
                if marker in seen:
                    R,t,err=pose(square(side),seen[marker],K,dist)
                    row[str(marker)]=dict(R=R.tolist(),t=t.tolist(),error_px=err)
            obj,img=[],[]
            cubeids=[i for i in decoded if i in pointmap]
            for i in cubeids:obj.extend(pointmap[i]);img.extend(seen[i])
            if obj:
                R,t,err=pose(np.asarray(obj),np.asarray(img),K,dist)
                row['cube']=dict(R=R.tolist(),t=t.tolist(),error_px=err,marker_ids=cubeids,
                    planar_single_marker_ambiguity=len(cubeids)==1)
            frames.append(row)
        empty=[r for r in frames if '40' in r and '41' in r and not r.get('cube')]
        empty.sort(key=lambda r:(r['image']!='stage_0_end.png',r['40']['error_px']+r['41']['error_px']))
        converted=[]
        if empty:
            reference=empty[0];Rh=np.array(reference['40']['R']);th=np.array(reference['40']['t'])
            Ra=np.array(reference['41']['R']);ta=np.array(reference['41']['t'])
            for row in frames:
                if '41' not in row or 'cube' not in row:continue
                R1,t1=np.array(row['41']['R']),np.array(row['41']['t'])
                Rc,tc=np.array(row['cube']['R']),np.array(row['cube']['t'])
                canchor=R1.T@(tc-t1);cempty=ta+Ra@canchor
                local=Rh.T@(cempty-th);Rlocal=Rh.T@Ra@R1.T@Rc
                world=np.array([.11,0,.03])+Rw@(local-g)
                converted.append(dict(image=row['image'],center_world_m=world.tolist(),
                    rotation_world=(Rw@Rlocal).tolist(),cube_reprojection_px=row['cube']['error_px'],
                    cube_marker_ids=row['cube']['marker_ids'],
                    empty_reference=reference['image']))
        result=dict(folder=foldername,empty_pair_count=len(empty),frames=frames,registered=converted,
            cube_side_mm=55,cube_marker_side_mm=10.7,anchor_marker_side_mm=27,
            registration_verified=False,physical_90_degree_rotation_verified=False)
        folders.append(result)
        print(foldername,'empty_pairs',len(empty),'registered',[(r['image'],r['cube_marker_ids'],np.round(r['center_world_m'],3).tolist()) for r in converted],flush=True)
    (OUT/'same_session_marker_registration.json').write_text(json.dumps(dict(folders=folders,
        hand_marker_transform=reg,hardware_commands_sent=0,original_images_unchanged=True),indent=2),encoding='utf-8')


if __name__=='__main__':main()
