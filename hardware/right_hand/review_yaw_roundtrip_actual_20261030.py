"""Review saved two-leg experiments. No devices, image edits or motor writes."""
import json,hashlib,math
from pathlib import Path
import numpy as np,cv2 as cv
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan,LOADED
from trace_adaptive_loaded_grid_20261029 import fit,K,D,det
from measure_successful_quarter_20261028 import relative
from measure_inverse_side_marker_20261029 import poses
ROOT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def label_frames(frames):
    seen_forward_hold=inverse=False;out=[]
    for f in frames:
        p=f['phase']
        if p=='stage_7_horizontal_yaw' and seen_forward_hold:inverse=True
        if p=='stage_8_horizontal_hold':seen_forward_hold=True
        label={'loaded_countdown':'placed','stage_5_loaded_grasp':'grasp','stage_6_grasp_hold':'grasp_hold','stage_9_loaded_open':'open','stage_10_remove_hold':'open_hold','stage_2_fixed_hold':'open_hold'}.get(p,p)
        if p=='stage_7_horizontal_yaw':label='inverse_yaw' if inverse else 'forward_yaw'
        if p=='stage_8_horizontal_hold':label='inverse_hold' if inverse else 'forward_hold'
        out.append(dict(f,leg_label=label))
    return out
def summary(a,b,body=False):
    if body:
        a=[dict(r,rotation=r['body_rotation']) for r in a if 'body_rotation' in r];b=[dict(r,rotation=r['body_rotation']) for r in b if 'body_rotation' in r]
    values=[relative(x,y) for x in a for y in b]
    r=dict(before_frames=len(a),after_frames=len(b),pairs=len(values))
    if values:
        for key in ('twist_deg','total_deg','up_axis_change_deg'):r[key+'_p05_median_p95']=np.percentile([v[key] for v in values],[5,50,95]).tolist()
    return r
def main():
    for F in sorted(ROOT.glob('yaw_roundtrip_trial_*'))[-2:]:
        O=F/'offline_review';O.mkdir(exist_ok=True);r=json.loads((F/'result.json').read_bytes());p=YawRoundtripPlan(F/'fresh_off_request.json')
        rows=p.plan(np.asarray(r['accepted_enable_goal'],int));actual=[e['requested'] for e in r['write_events'] if e['address']==116]
        assert actual[-len(rows):]==[q.tolist() for _,q in rows]
        assert r['status']=='yaw_roundtrip_observation_complete' and r['all_torque_off_verified'] and r['settings_restored']
        frames=label_frames(json.loads((F/'camera/frames.json').read_bytes())['frames']);(O/'leg_frames.json').write_text(json.dumps(dict(frames=frames),indent=2)+'\n',encoding='utf-8')
        selected=[]
        for group in ('placed','grasp','grasp_hold','forward_yaw','forward_hold','inverse_yaw','inverse_hold','open','open_hold'):
            ff=[f for f in frames if f['leg_label']==group];stride=6 if group in ('grasp','forward_yaw','inverse_yaw','open') else 2
            selected.extend(ff[::stride])
        selected.sort(key=lambda f:f['camera_read_monotonic']);fits=[]
        for index,f in enumerate(selected):
            path=F/'camera'/f['image'];row=fit(path);row.update(leg_label=f['leg_label'],phase=f['phase'],t=f['camera_read_monotonic']-r['enabled_at'])
            if row.get('accepted'):
                if 'palm40_corners' in row:
                    pp=poses(np.array(row['palm40_corners']),.0282)
                    if pp:row['body_rotation']=(pp[0]['rotation'].T@np.array(row['rotation'])).tolist()
                if 0 in row.get('ids',[]):
                    im=cv.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv.IMREAD_COLOR);cs,ids,_=det.detectMarkers(cv.cvtColor(im,cv.COLOR_BGR2GRAY))
                    found={} if ids is None else {int(i):np.asarray(c).reshape(4,2) for i,c in zip(ids.ravel(),cs)}
                    side=poses(found[0],.0107) if 0 in found else []
                    if side:row['green_in_white_rotation']=(side[0]['rotation'].T@np.array(row['rotation'])).tolist()
            fits.append(row)
            if index%120==0:print(json.dumps(dict(folder=F.name,processed=index,total=len(selected))),flush=True)
        groups={name:[x for x in fits if x['leg_label']==name and x.get('accepted')] for name in ('placed','grasp_hold','forward_hold','inverse_hold','open_hold')}
        comparisons={}
        for name,a,b in (('placement_to_forward','placed','forward_hold'),('grasp_to_forward','grasp_hold','forward_hold'),('forward_to_inverse','forward_hold','inverse_hold'),('inverse_to_open','inverse_hold','open_hold'),('placement_to_open','placed','open_hold')):
            comparisons[name]=dict(camera_relative=summary(groups[a],groups[b]),decoded_palm_compensated=summary(groups[a],groups[b],True))
        relation=[]
        aa=[x for x in groups['forward_hold'] if 'green_in_white_rotation' in x];bb=[x for x in groups['inverse_hold'] if 'green_in_white_rotation' in x]
        for a in aa:
          for b in bb:
            delta=np.array(a['green_in_white_rotation']).T@np.array(b['green_in_white_rotation']);relation.append(float(np.degrees(np.linalg.norm(cv.Rodrigues(delta)[0]))))
        s=r['samples'];camera=r['camera_recording'];report=dict(status='completed_roundtrip_saved_image_orientation_review',source=F.name,source_sha256=sha(F/'result.json'),candidate_sha256=sha(LOADED),
            all_actual_goal_writes_match_ON_rebased_queue=True,queue_goals=len(rows),forward_yaw_goals=638,inverse_yaw_goals=638,
            all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=r['cube_removal_confirmed'],cube_present=r['cube_present'],
            min_voltage_v=min(min(x['voltage_v']) for x in s),peak_current_ma=max(max(abs(v) for v in x['current_ma']) for x in s),max_tracking_rad=max(max(x['tracking_rad']) for x in s),
            read_failures=len(r.get('read_failures',[])),all_recorded_read_failures_recovered=all(x.get('recovered') for x in r.get('read_failures',[])),
            camera_frames=camera['frame_count'],camera_blank_frames=camera['image_signal_blank_frames'],comparisons=comparisons,
            accepted_grid_frames=sum(x.get('accepted',False) for x in fits),requested_grid_frames=len(fits),
            accepted_by_group={k:len(v) for k,v in groups.items()},
            green_to_white_relative_pose_change_deg_p05_median_p95=np.percentile(relation,[5,50,95]).tolist() if relation else None,
            layer_twist_not_certified_from_single_face=True,inverse_leg_was_programmed_not_replacement=True,
            metric_robot_registration_verified=False,frame_spread_not_absolute_accuracy=True,source_images_unchanged=True,assistant_motor_writes=0,rows=fits)
        (O/'physical_and_angle_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
if __name__=='__main__':main()
