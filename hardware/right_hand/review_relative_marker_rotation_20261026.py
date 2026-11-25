"""Exercise relative rotation on known synthetic motions and saved evidence."""
import json
from pathlib import Path
import cv2,numpy as np
from relative_marker_rotation_20261026 import compare_marker_frames,detect_markers
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026'

def R(v):return cv2.Rodrigues(np.asarray(v,float))[0]

def synthetic(K,dist,yaw_deg,side,camera_moves=False):
    h=side/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],float)
    Ra=R([2.4,.3,.1]);ta=np.array([-.01,.02,.6]);D=R([0,0,np.radians(yaw_deg)])
    mounts={2:(R([0,0,.08]),np.array([.005,.002,.0875])),5:(R([np.pi/2,0,0])@R([0,0,-.15]),np.array([.002,-.0275,.061]))}
    def project(B,t):return cv2.projectPoints(obj,cv2.Rodrigues(B)[0],t,K,dist)[0].reshape(4,2).tolist()
    Q=R([.03,-.02,.01]) if camera_moves else np.eye(3);Ra2=Q@Ra;ta2=Q@ta+(np.array([.01,-.005,.002]) if camera_moves else 0.)
    before={41:project(Ra,ta)};after={41:project(Ra2,ta2)}
    for i,(B,p) in mounts.items():
        before[i]=project(Ra@B,ta+Ra@p)
        after[i]=project(Ra2@D@B,ta2+Ra2@(D@p))
    return before,after

def main():
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];checks=[]
    for camera_moves,angle in [(moves,angle) for moves in (False,True) for angle in (-90.,0.,90.)]:
        before,after=synthetic(K,dist,angle,.0107,camera_moves)
        estimates=[]
        for scale in (.0105,.0106,.0107):
            out=compare_marker_frames(before,after,41,[2,5],K,dist,scale,top_marker_id=2,max_reference_pose_change_deg=15);estimate=out['best']['reference_axis_twist_deg'];error=abs((estimate-angle+180)%360-180)
            # The exact-data optimum must recover the known motion. Conservative
            # branch ambiguity remains an independent reason to decline a
            # measurement; do not weaken it merely to make this check pass.
            assert error<.001,(angle,scale,out)
            estimates.append(estimate)
        checks.append(dict(camera_moves=camera_moves,known_angle_deg=angle,estimates_for_10_5_10_6_10_7mm=estimates,max_error_deg=max(abs((v-angle+180)%360-180) for v in estimates),conservative_branch_qualification=out['qualified'],near_best_hypothesis_yaw_spread_deg=out['near_best_hypothesis_yaw_spread_deg']))
    cases=[];folder=ROOT/'sequence_vision_20261011_051517_532925'
    for first,last in [('stage_4_eligible.png','stage_6_eligible.png'),('stage_4_end.png','stage_6_eligible.png')]:
        a=cv2.imdecode(np.fromfile(folder/first,np.uint8),cv2.IMREAD_COLOR);b=cv2.imdecode(np.fromfile(folder/last,np.uint8),cv2.IMREAD_COLOR)
        if a is None or b is None:continue
        da,db=detect_markers(a),detect_markers(b);out=compare_marker_frames(da,db,40,[0,3],K,dist,up_from_cross=(0,3),max_reference_pose_change_deg=15)
        cases.append(dict(before=str((folder/first).relative_to(ROOT)),after=str((folder/last).relative_to(ROOT)),detected_before=sorted(da),detected_after=sorted(db),**out))
    result=dict(status='scale_independent_relative_rotation_saved_evidence_review',synthetic_known_angle_checks=checks,cases=cases,not_an_absolute_pose_calibration=True,not_live_SAC_or_camera_controller=True,hardware_execution_allowed=False,real_devices_opened=False);p=OUT/'relative_rotation_review.json';p.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
