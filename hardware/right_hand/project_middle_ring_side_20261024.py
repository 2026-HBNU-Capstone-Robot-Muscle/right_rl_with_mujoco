"""Independent saved-photo overlays; no joint fitting and no device access."""
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/middle_ring_side_review'


def main():
    rows=json.loads((OUT/'paired_native_geometry.json').read_bytes())['captures']
    with np.load(ROOT/'camera1_720.npz') as z:K,dist=z['K'],z['dist']
    params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
    detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),params)
    registration=candidate()
    Rpm=np.diag([1.,-1.,-1.])@np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
    colors={'if_':(255,130,50),'mf_':(30,220,30),'rf_':(30,170,245),'th_':(50,50,240),'palm':(180,180,180)}
    output=[];panel=np.zeros((1440,2560,3),np.uint8)
    for index,g in enumerate(rows):
        folder=ROOT/g['folder'];im=cv.imdecode(np.frombuffer((folder/'paired_photo.png').read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));ids=[] if ids is None else ids.ravel().tolist()
        branches=marker_solutions(corners,ids,40,.0282,K,dist)
        Rwm=np.array(g['palm_rotation_world'])@Rpm
        marker_world=np.array(g['middle_base_position_world_m'])-Rwm@np.array(registration['motor_axis_landmarks_in_marker_m']['9'])
        results=[]
        for branch,(Rcm,tcm,error) in enumerate(branches):
            Rcw=Rcm@Rwm.T;tcw=tcm-Rcw@marker_world;rv=cv.Rodrigues(Rcw)[0];show=im.copy()
            def project(points):
                return cv.projectPoints(np.asarray(points,np.float64),rv,tcw,K,dist)[0].reshape(-1,2)
            for mesh in g['meshes']:
                points=np.array(mesh['vertices_world_m']);px=project(points)
                if np.any((points@Rcw.T+tcw)[:,2]<=0) or not np.isfinite(px).all() or max(abs(px.ravel()))>6000:continue
                color=next((v for key,v in colors.items() if mesh['body'].startswith(key)),(180,180,180))
                cv.polylines(show,[cv.convexHull(px.astype(np.float32)).astype(np.int32)],True,color,1,cv.LINE_AA)
            projections=[]
            for joint in g['joints']:
                xy=project([joint['world_m']])[0]
                if not np.isfinite(xy).all() or max(abs(xy))>6000:continue
                cv.circle(show,tuple(np.rint(xy).astype(int)),3,(255,255,255),-1)
                cv.putText(show,str(joint['motor_id']),tuple(np.rint(xy+[5,-6]).astype(int)),cv.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv.LINE_AA)
                projections.append(dict(motor_id=joint['motor_id'],name=joint['name'],pixel=xy.tolist()))
            cv.rectangle(show,(0,0),(1280,54),(0,0,0),-1)
            cv.putText(show,f'Saved paired OFF pose | marker branch {branch} fit {error:.2f}px | no joint zero fitting',(8,20),cv.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),1)
            cv.putText(show,'Green=middle Gold=ring Red=thumb Blue=index. Legacy rigid marker transform is provisional.',(8,43),cv.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
            name=f'side_{index+1}_overlay_branch_{branch}.jpg'
            (OUT/name).write_bytes(cv.imencode('.jpg',show,[cv.IMWRITE_JPEG_QUALITY,94])[1].tobytes())
            if branch<2:panel[index*720:(index+1)*720,branch*1280:(branch+1)*1280]=show
            results.append(dict(branch=branch,marker_fit_px=error,world_to_camera_rotation=Rcw.tolist(),
                world_to_camera_translation_m=tcw.tolist(),joint_projections=projections,overlay=name))
        output.append(dict(folder=g['folder'],detected_ids=ids,branches=results,
            raw_ticks=g['raw_ticks'],reference_transform_source='hand_registration_geometry.candidate; historical approximate measurements'))
        print('OVERLAY',g['folder'],'ids',ids,'branches',[(b['branch'],round(b['marker_fit_px'],3)) for b in results],flush=True)
    (OUT/'two_views_two_marker_branches.jpg').write_bytes(cv.imencode('.jpg',panel,[cv.IMWRITE_JPEG_QUALITY,92])[1].tobytes())
    (OUT/'projection_diagnostic.json').write_text(json.dumps(dict(captures=output,registration=registration,
        joint_zeros_fitted=False,joint_ranges_changed=False,geometry_calibration_verified=False,
        hardware_execution_allowed=False,hardware_commands_sent=0),indent=2),encoding='utf-8')


if __name__=='__main__':main()
