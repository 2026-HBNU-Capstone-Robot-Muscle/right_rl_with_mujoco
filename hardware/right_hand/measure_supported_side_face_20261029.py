"""Same-face rotation from saved WhiteID0; infer face-up from visible Green.

The four possible printed marker axes are tested against the Green sticker
centroid in each image. No unreadable top marker or robot registration is
invented. Frame spread is not absolute measurement accuracy.
"""
import json,hashlib,math
from pathlib import Path
import cv2 as cv,numpy as np
from measure_inverse_side_marker_20261029 import poses,twist,K,D,det
ROOT=Path(__file__).resolve().parent;F=ROOT/'supported_inverse_trial_20261029_231345_341471';O=F/'offline_review'
AXES=np.array([[1.,0,0],[-1.,0,0],[0,1.,0],[0,-1.,0]])

def face_up_vote(image,pose,points):
    R=np.asarray(pose['rotation']);t=np.asarray(pose['translation']);rv=cv.Rodrigues(R)[0]
    square=np.array([[-.042,-.042,0],[.042,-.042,0],[.042,.042,0],[-.042,.042,0]],float)
    polygon=cv.projectPoints(square,rv,t,K,D)[0].reshape(-1,2);mask=np.zeros(image.shape[:2],np.uint8)
    cv.fillConvexPoly(mask,np.rint(polygon).astype(np.int32),255)
    hsv=cv.cvtColor(image,cv.COLOR_BGR2HSV)
    green=cv.bitwise_and(cv.inRange(hsv,np.array([35,45,150]),np.array([100,255,255])),mask)
    n,labels,stats,centers=cv.connectedComponentsWithStats(green);kept=[i for i in range(1,n) if 100<=stats[i,4]<=8000]
    if len(kept)<2:return None
    center=np.average(centers[kept],axis=0,weights=stats[kept,4]);origin=cv.projectPoints(np.zeros((1,3)),rv,t,K,D)[0].reshape(2)
    delta=center-origin
    if np.linalg.norm(delta)<20:return None
    delta/=np.linalg.norm(delta);projected=cv.projectPoints(AXES*.012,rv,t,K,D)[0].reshape(-1,2)-origin
    projected/=np.linalg.norm(projected,axis=1)[:,None];scores=projected@delta;order=np.argsort(scores)[::-1]
    if scores[order[0]]<.65 or scores[order[0]]-scores[order[1]]<.15:return None
    return dict(axis_index=int(order[0]),cosine=float(scores[order[0]]),margin=float(scores[order[0]]-scores[order[1]]),green_components=len(kept))

def choose(available,previous):
    near=[p for p in available if p['rmse_px']<=available[0]['rmse_px']+.25]
    return min(near,key=lambda p:np.linalg.norm(cv.Rodrigues(previous.T@p['rotation'])[0])) if previous is not None else available[0]

def main():
    O.mkdir(exist_ok=True);run=json.loads((F/'result.json').read_bytes());frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
    start=next(i for i,r in enumerate(frames) if r['phase']=='loaded_countdown');end=max(i for i,r in enumerate(frames) if r['phase']=='stage_10_remove_hold')
    rows=[];last_side=last_palm=None
    for index in range(start,end+1,3):
        f=frames[index];path=F/'camera'/f['image'];image=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR)
        corners,ids,_=det.detectMarkers(cv.cvtColor(image,cv.COLOR_BGR2GRAY));found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),corners)}
        row=dict(image=f['image'],phase=f['phase'],t=f['camera_read_monotonic']-run['enabled_at'],decoded_ids=list(found))
        if 0 in found:
            candidates=poses(found[0],.0107)
            if candidates:
                side=choose(candidates,last_side);last_side=side['rotation'];row.update(side_rotation=last_side.tolist(),side_RMSE_px=side['rmse_px'])
                vote=face_up_vote(image,side,found[0])
                if vote:row['visible_green_face_up_vote']=vote
        if 40 in found:
            candidates=poses(found[40],.0282)
            if candidates:
                palm=choose(candidates,last_palm);last_palm=palm['rotation'];row.update(palm_rotation=last_palm.tolist(),palm_RMSE_px=palm['rmse_px'],palm_method='decoded_ID40')
        rows.append(row)
    before=[r for r in rows if r['phase'] in ('loaded_countdown','stage_6_grasp_hold') and 'visible_green_face_up_vote' in r]
    votes=[r['visible_green_face_up_vote']['axis_index'] for r in before]
    if len(votes)<3:raise RuntimeError('Face-up image validation insufficient; no angle claim')
    counts=np.bincount(votes,minlength=4);winning=int(np.argmax(counts));axis=AXES[winning];purity=float(counts[winning]/sum(counts))
    if purity<.9:raise RuntimeError('Printed marker face-up axis is ambiguous')
    aftervotes=[r['visible_green_face_up_vote']['axis_index'] for r in rows if r['phase'] in ('stage_8_horizontal_hold','stage_10_remove_hold') and 'visible_green_face_up_vote' in r]
    stable_after=bool(len(aftervotes)>=3 and np.mean(np.asarray(aftervotes)==winning)>=.9)
    comparisons={}
    for name,a0,b0 in (('placement_to_turn','loaded_countdown','stage_8_horizontal_hold'),('grasp_to_turn','stage_6_grasp_hold','stage_8_horizontal_hold'),('placement_to_open','loaded_countdown','stage_10_remove_hold')):
        aa=[r for r in rows if r['phase']==a0 and 'side_rotation' in r];bb=[r for r in rows if r['phase']==b0 and 'side_rotation' in r]
        camera=[];corrected=[];tilt=[]
        for a in aa:
          for b in bb:
            ra,rb=np.asarray(a['side_rotation']),np.asarray(b['side_rotation']);camera.append(twist(ra.T@rb,axis))
            if 'palm_rotation' in a and 'palm_rotation' in b:
                ca=np.asarray(a['palm_rotation']).T@ra;cb=np.asarray(b['palm_rotation']).T@rb
                corrected.append(twist(ca.T@cb,axis));tilt.append(math.degrees(math.acos(float(np.clip((ca@axis)@(cb@axis),-1,1)))))
        comparisons[name]=dict(before_frames=len(aa),after_frames=len(bb),palm_pairs=len(corrected),
            camera_twist_p05_median_p95_deg=np.percentile(camera,[5,50,95]).tolist() if camera else None,
            palm_relative_twist_p05_median_p95_deg=np.percentile(corrected,[5,50,95]).tolist() if corrected else None,
            palm_relative_up_change_p05_median_p95_deg=np.percentile(tilt,[5,50,95]).tolist() if tilt else None)
    references=[r for r in rows if r['phase']=='loaded_countdown' and 'side_rotation' in r and 'palm_rotation' in r]
    if not references:raise RuntimeError('Missing before reference')
    ref=references[len(references)//2];base=np.asarray(ref['palm_rotation']).T@np.asarray(ref['side_rotation'])
    for row in rows:
        if 'side_rotation' in row and 'palm_rotation' in row:
            relative=np.asarray(row['palm_rotation']).T@np.asarray(row['side_rotation']);row['yaw_from_placement_deg']=twist(base.T@relative,axis)
    turns=[r for r in rows if r['phase']=='stage_7_horizontal_yaw' and 'yaw_from_placement_deg' in r]
    report=dict(status='same_side0_rotation_with_image_validated_face_up_axis',source=F.name,source_result_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),
        green_axis_in_ID0=axis.tolist(),before_axis_votes=counts.tolist(),before_axis_vote_purity=purity,after_axis_votes=aftervotes,after_axis_consistent=stable_after,
        comparisons=comparisons,turn_trace_range_deg=[min(r['yaw_from_placement_deg'] for r in turns),max(r['yaw_from_placement_deg'] for r in turns)] if turns else None,
        rows=rows,green_top_marker_not_required=True,palm_compensation_uses_only_decoded_ID40=True,
        certified_robot_registration=False,frame_spread_not_absolute_accuracy=True,source_images_unchanged=True,motor_writes=0)
    (O/'side_face_rotation_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
if __name__=='__main__':main()
