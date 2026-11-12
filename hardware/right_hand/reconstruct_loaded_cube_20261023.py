"""Same-run marker bridge and cube poses, for offline model diagnosis only."""
import itertools
import json
from pathlib import Path
import sys
import cv2 as cv
import numpy as np
from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions
from marker_detector import MarkerDetector

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'
sys.path.insert(0,str(ROOT.parent/'work/Leap_Test/vision/rubik_center'))
import cube_board


def image(path):return cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)


def main():
    pairs=json.loads((FOLDER/'same_run_reference_pair_scan.json').read_bytes())['rows']
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    bridge=[]
    for f in pairs:
        ids=[40,41];cs=[np.array(f['corners'][str(i)]) for i in ids]
        a=marker_solutions(cs,ids,40,.0282,K,dist)
        b=marker_solutions(cs,ids,41,.027,K,dist)
        for (ih,(Rh,th,eh)),(ia,(Ra,ta,ea)) in itertools.product(enumerate(a),enumerate(b)):
            bridge.append(dict(image=f['image'],phase=f['phase'],indices=[ih,ia],
                R_41_40=(Ra.T@Rh).tolist(),t_41_40=(Ra.T@(th-ta)).tolist(),
                errors_px=[eh,ea]))
    dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    detector=MarkerDetector(dictionary,[0,1,2,3,4,5,40],hand_id=41,required_cube_count=2)
    frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames']
    selected=[]
    for phase in ('stage_3_loaded_approach','stage_4_supported_cube_rotation'):
        group=[f for f in frames if f['phase']==phase]
        selected += [group[i] for i in sorted(set([0,len(group)//2,len(group)-1]))]
    reg=candidate();Rw=np.asarray(reg['hand_to_world_candidate'])
    grasp=np.asarray(reg['grasp_site_in_marker_m_candidate']);grasp_world=np.array([.11,0,.03])
    rows=[]
    for f in selected:
        im=image(FOLDER/'camera'/f['image']);gray=cv.cvtColor(im,cv.COLOR_BGR2GRAY)
        cs,ids,_=detector.detectMarkers(gray)
        ids=[] if ids is None else list(map(int,ids.ravel()))
        points={i:np.asarray(c).reshape(4,2) for i,c in zip(ids,cs)}
        # A visible small upper marker can be lost among the large references.
        # Detect the upper cube ROI separately, retaining only decoded ID5.
        roi=gray[50:260,450:900]
        rc,ri,_=detector.detectMarkers(roi)
        if ri is not None:
            for c,i in zip(rc,ri.ravel()):
                if int(i)==5 and 5 not in points:points[5]=np.asarray(c).reshape(4,2)+[450,50]
        ids=list(points);cs=[points[i] for i in ids]
        anchored=marker_solutions(cs,ids,41,.027,K,dist)
        cube_ids=sorted(i for i in ids if i<6);solutions=[]
        if len(cube_ids)>=2:
            for spins in itertools.product(range(4),repeat=len(cube_ids)):
                rot=[0]*6
                for i,s in zip(cube_ids,spins):rot[i]=s
                objs,_=cube_board.object_points(cube_mm=55,marker_mm=10.7,layout='centers',face_rot=rot)
                op=np.concatenate([objs[i] for i in cube_ids]).astype(np.float64)
                ip=np.concatenate([points[i] for i in cube_ids]).astype(np.float64)
                ok,rv,tv=cv.solvePnP(op,ip,K,dist,flags=cv.SOLVEPNP_SQPNP)
                if not ok:continue
                rv,tv=cv.solvePnPRefineLM(op,ip,K,dist,rv,tv)
                R=cv.Rodrigues(rv)[0]
                if np.min((op@R.T+tv.ravel())[:,2])<=0:continue
                proj=cv.projectPoints(op,rv,tv,K,dist)[0].reshape(-1,2)
                solutions.append(dict(R=R.tolist(),t=tv.ravel().tolist(),face_spins=rot,
                    error_px=float(np.mean(np.linalg.norm(proj-ip,axis=1)))))
            solutions.sort(key=lambda x:x['error_px']);solutions=solutions[:2]
        elif cube_ids:
            ident=cube_ids[0];n,u,v=map(np.array,cube_board.FACES[ident])
            for spin in range(4):
                rotations=[0]*6;rotations[ident]=spin
                objs,_=cube_board.object_points(cube_mm=55,marker_mm=10.7,layout='centers',face_rot=rotations)
                # SQPNP / iterative polishing for known cube-centered points.
                ok,rv,tv=cv.solvePnP(np.array(objs[ident],np.float64),points[ident].astype(np.float64),K,dist,flags=cv.SOLVEPNP_SQPNP)
                if not ok:continue
                rv,tv=cv.solvePnPRefineLM(np.array(objs[ident],np.float64),points[ident].astype(np.float64),K,dist,rv,tv)
                R=cv.Rodrigues(rv)[0];proj=cv.projectPoints(np.array(objs[ident],np.float64),rv,tv,K,dist)[0].reshape(-1,2)
                solutions.append(dict(R=R.tolist(),t=tv.ravel().tolist(),face_spins=rotations,
                    error_px=float(np.mean(np.linalg.norm(proj-points[ident],axis=1)))))
        hypotheses=[]
        for b,(ia,(Ra,ta,ea)),(ic,c) in itertools.product(bridge,enumerate(anchored),enumerate(solutions)):
            Rb=np.array(b['R_41_40']);tb=np.array(b['t_41_40'])
            Rcw=Ra@Rb@Rw.T;tcw=ta+Ra@(tb+Rb@grasp)-Rcw@grasp_world
            Rc=np.array(c['R']);tc=np.array(c['t'])
            hypotheses.append(dict(bridge_image=b['image'],bridge_indices=b['indices'],anchor_branch=ia,cube_solution=ic,
                error_px=sum(b['errors_px'])+ea+c['error_px'],
                world_to_camera_rotation=Rcw.tolist(),world_to_camera_translation=tcw.tolist(),
                cube_center_world_m=(Rcw.T@(tc-tcw)).tolist(),cube_rotation_world=(Rcw.T@Rc).tolist()))
        rows.append(dict(**f,ids=ids,corners={str(i):p.tolist() for i,p in points.items()},
                         cube_pose_candidates=solutions,hypotheses=hypotheses))
        print(f['phase'],f['image'],'IDs',ids,'cube fits',[round(c['error_px'],2) for c in solutions],
              'hypotheses',len(hypotheses),flush=True)
    out=dict(status='same_run_conditional_cube_registration',marker40_mm=28.2,marker41_mm=27,
        cube_mm=55,cube_marker_mm=10.7,bridge=bridge,rows=rows,registration_verified=False,
        limitations=['Marker-to-hand mounting dimensions and intrinsics remain approximate.',
                     'Planar branches and center-sticker spin are retained as competing hypotheses.',
                     'Diagnostic only; never install a transform or a motor goal from this result.'])
    (FOLDER/'conditional_cube_registration.json').write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
