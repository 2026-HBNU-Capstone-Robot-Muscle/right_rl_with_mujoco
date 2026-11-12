"""Camera-relative orientation of the same decoded green ID5; saved data only."""
import json,math
from pathlib import Path
import cv2 as cv,numpy as np
from marker_detector import MarkerDetector
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'horizontal_quarter_trial_20261028_012305_894520'
OUT=ROOT/'right_hand_rotation_20261028/completed_quarter_012305'
z=np.load(ROOT/'camera1_720.npz');K,D=z['K'],z['dist']
obj=np.float64([[-.00535,.00535,0],[.00535,.00535,0],[.00535,-.00535,0],[-.00535,-.00535,0]])
det=MarkerDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),range(6),required_cube_count=1)
frames=json.loads((FOLDER/'camera/frames.json').read_bytes())['frames'];rows=[]
for phase in ('loaded_countdown','stage_6_grasp_hold','stage_8_horizontal_hold','stage_10_remove_hold'):
    for f in [v for v in frames if v['phase']==phase][-18:]:
        image=cv.imdecode(np.frombuffer((FOLDER/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=det.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY))
        found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),corners)}
        row=dict(image=f['image'],phase=phase,ids=list(found),candidates=[])
        if 5 in found:
            ok,rvs,tvs,errs=cv.solvePnPGeneric(obj,found[5].astype(np.float64),K,D,flags=cv.SOLVEPNP_IPPE_SQUARE)
            for rv,tv in zip(rvs,tvs):
                projection=cv.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
                error=float(np.sqrt(np.mean(np.sum((projection-found[5])**2,axis=1))))
                if float(tv[2,0])>0:row['candidates'].append(dict(R=cv.Rodrigues(rv)[0].tolist(),rmse_px=error))
            row['candidates'].sort(key=lambda p:p['rmse_px'])
        rows.append(row)
def compare(phase0,phase1):
    before=[v for v in rows if v['phase']==phase0 and v['candidates'] and v['candidates'][0]['rmse_px']<1.]
    after=[v for v in rows if v['phase']==phase1 and v['candidates'] and v['candidates'][0]['rmse_px']<1.]
    values=[]
    for b in before:
        for a in after:
            rb=np.array(b['candidates'][0]['R']);ra=np.array(a['candidates'][0]['R']);r=rb.T@ra;v=cv.Rodrigues(r)[0].ravel();theta=np.linalg.norm(v)
            qz=v[2]/theta*math.sin(theta/2) if theta>1e-9 else 0
            values.append([math.degrees(2*math.atan2(qz,math.cos(theta/2))),math.degrees(theta),math.degrees(math.acos(float(np.clip(rb[:,2]@ra[:,2],-1,1))))])
    return dict(before_frames=len(before),after_frames=len(after),
        twist_total_tilt_p05_median_p95=np.percentile(values,[5,50,95],axis=0).tolist() if values else None)
comparisons=dict(placement_to_turn=compare('loaded_countdown','stage_8_horizontal_hold'),
    grasp_to_turn=compare('stage_6_grasp_hold','stage_8_horizontal_hold'),
    placement_to_open=compare('loaded_countdown','stage_10_remove_hold'))
def load(name):return cv.imdecode(np.frombuffer((FOLDER/name).read_bytes(),np.uint8),cv.IMREAD_GRAYSCALE)
a=load('cube_before_grasp.jpg');b=load('cube_horizontal_turn_hold.jpg');mask=np.zeros(a.shape,np.uint8);mask[380:700,290:480]=255
sift=cv.SIFT_create(nfeatures=1500);ka,da=sift.detectAndCompute(a,mask);kb,db=sift.detectAndCompute(b,mask)
matches=[m for m,n in cv.BFMatcher().knnMatch(da,db,k=2) if m.distance<.7*n.distance]
camera=dict(ratio_matches=len(matches),ROI='palm/wrist housing; excludes cube and most human skin',robot_registration_verified=False)
if len(matches)>=10:
    p=np.float32([ka[m.queryIdx].pt for m in matches]);q=np.float32([kb[m.trainIdx].pt for m in matches]);h,keep=cv.findHomography(p,q,cv.RANSAC,1.5)
    if h is not None:
        selected=keep.ravel().astype(bool);camera.update(inliers=int(selected.sum()),shift_px_median_p95=np.percentile(np.linalg.norm(p[selected]-q[selected],axis=1),[50,95]).tolist(),homography=h.tolist())
report=dict(source=FOLDER.name,method='same_green_ID5_positive_depth_IPPE_best_reprojection',comparisons=comparisons,
    static_palm_feature_check=camera,frames=rows,certified_world_or_robot_frame=False,
    limitation='Small single marker, planar ambiguity and camera calibration can bias estimates. Percentiles describe sample spread, not absolute angle accuracy.',motor_writes=0)
(OUT/'same_marker_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(comparisons=comparisons,static_palm_feature_check=camera)))
