"""Project native geometry on a SAVED paired photo using provisional marker registration."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np
from hand_registration_geometry import candidate
from review_prepared_cube_20261022 import marker_solutions

ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'pose_pair_readonly_20261024_045449_430810'
geometry=json.loads((FOLDER/'native_geometry.json').read_bytes())
photo_blob=(FOLDER/'paired_photo.png').read_bytes()
im=cv.imdecode(np.frombuffer(photo_blob,np.uint8),cv.IMREAD_COLOR)
parameters=cv.aruco.DetectorParameters();parameters.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX
detector=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),parameters)
corners,ids,_=detector.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY));ids=[] if ids is None else ids.ravel().tolist()
with np.load(ROOT/'camera1_720.npz') as data:K,dist=data['K'],data['dist']
solutions=marker_solutions(corners,ids,40,.0282,K,dist)
if not solutions:raise RuntimeError('No ID40 pose; do not infer camera registration')
registration=candidate()
Rpm=np.diag([1.,-1.,-1.])@np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
Rwm=np.array(geometry['palm_rotation_world'])@Rpm
marker_world=np.array(geometry['middle_base_position_world_m'])-Rwm@np.array(registration['motor_axis_landmarks_in_marker_m']['9'])
colors={'if_':(255,130,50),'mf_':(30,220,30),'rf_':(30,170,245),'th_':(50,50,240),'palm':(180,180,180)}
branches=[]
for branch,(Rcm,tcm,error) in enumerate(solutions):
    Rcw=Rcm@Rwm.T;tcw=tcm-Rcw@marker_world;rv=cv.Rodrigues(Rcw)[0];show=im.copy()
    def project(points):return cv.projectPoints(np.array(points,np.float64),rv,tcw,K,dist)[0].reshape(-1,2)
    for mesh in geometry['meshes']:
        points=np.array(mesh['vertices_world_m']);px=project(points)
        if np.any((points@Rcw.T+tcw)[:,2]<=0) or not np.isfinite(px).all() or np.max(abs(px))>6000:continue
        col=next((v for key,v in colors.items() if mesh['body'].startswith(key)),(180,180,180))
        hull=cv.convexHull(px.astype(np.float32)).astype(np.int32)
        cv.polylines(show,[hull],True,col,1,cv.LINE_AA)
    joints=[]
    for joint in geometry['joints']:
        xy=project([joint['world_m']])[0]
        if not np.isfinite(xy).all() or max(abs(xy))>6000:continue
        point=tuple(np.rint(xy).astype(int));ident=joint['motor_id']
        cv.circle(show,point,3,(255,255,255),-1)
        cv.putText(show,str(ident),tuple(np.rint(xy+[4,-4]).astype(int)),cv.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1,cv.LINE_AA)
        joints.append(dict(motor_id=ident,name=joint['name'],projected_px=xy.tolist()))
    cv.rectangle(show,(0,0),(1280,47),(0,0,0),-1)
    cv.putText(show,f'Paired OFF encoders; provisional marker branch {branch}, marker fit {error:.3f}px',(8,18),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    cv.putText(show,'Blue=index Green=middle Gold=ring Red=thumb; no joint fitting; NOT calibration approval',(8,39),cv.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
    output=FOLDER/f'provisional_model_overlay_{branch}.png'
    output.write_bytes(cv.imencode('.png',show)[1].tobytes())
    branches.append(dict(branch=branch,marker_fit_px=error,world_to_camera_rotation=Rcw.tolist(),
        world_to_camera_translation=tcw.tolist(),joint_projections=joints,overlay=output.name))
record=dict(source=FOLDER.name,photo_sha256=hashlib.sha256(photo_blob).hexdigest(),
    geometry_source_sha256=geometry['source_sha256'],detected_ids=ids,branches=branches,
    registration=registration,registration_verified=False,joint_offsets_fitted=False,
    hardware_execution_allowed=False,real_devices_opened=False,
    limitation='Historical marker/axis labels predate wiring correction. These overlays are conditional diagnostics, not fitted calibration.')
(FOLDER/'provisional_projection_review.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
print('IDS',ids,'BRANCHES',[(v['branch'],v['marker_fit_px'],v['overlay']) for v in branches])
