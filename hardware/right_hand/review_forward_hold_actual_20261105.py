"""Analyze saved photographs/encoders and export real-time evidence. No devices."""
import hashlib, json, math
from collections import Counter
from pathlib import Path
import cv2 as cv
import numpy as np
from trace_adaptive_loaded_grid_20261029 import fit
from measure_successful_quarter_20261028 import relative
from measure_inverse_side_marker_20261029 import poses

ROOT = Path(__file__).resolve().parent
F = ROOT/'forward_hold_observe_20261105_030930_516359'
OUT = ROOT/'right_hand_rotation_20261104/forward_hold/actual_review_20261105'

def read(p):
    return cv.imdecode(np.frombuffer(p.read_bytes(), np.uint8), cv.IMREAD_COLOR)

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def contact_sheet(frames, enabled, phases, output):
    # Follow this run's phase times; do not reuse another run's elapsed times.
    start=phases['stage_7_horizontal_yaw']['first_t'];end=phases['stage_7_horizontal_yaw']['last_t']
    hold=phases['stage_8_horizontal_hold']
    times=[phases['stage_6_grasp_hold']['first_t'],*np.linspace(start,end,5).tolist(),(hold['first_t']+hold['last_t'])/2,hold['last_t']]
    canvas=np.full((3*240,3*426,3),245,np.uint8);ft=np.array([f['camera_read_monotonic']-enabled for f in frames]);selections=[]
    for n,t in enumerate(times):
        f=frames[int(np.argmin(abs(ft-t)))];im=read(F/'camera'/f['image']);small=cv.resize(im,(426,240));actual=f['camera_read_monotonic']-enabled
        cv.putText(small,f't = {actual:.1f}s',(10,24),cv.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
        y,x=divmod(n,3);canvas[y*240:(y+1)*240,x*426:(x+1)*426]=small
        selections.append(dict(image=f['image'],phase=f['phase'],target_t=float(t),actual_t=float(actual)))
    if any(v['phase'] not in ('stage_8_horizontal_hold','stage_2_fixed_hold') for v in selections[-2:]):raise RuntimeError('Hold contact sheet selected an unrelated phase')
    ok,data=cv.imencode('.jpg',canvas)
    if not ok:raise RuntimeError('Contact sheet encode failed')
    output.write_bytes(data.tobytes())
    output.with_suffix('.json').write_text(json.dumps(selections,indent=2)+'\n',encoding='utf-8')
    return selections

def stats(a, b, body=False):
    if body:
        a=[dict(r,rotation=r['body_rotation']) for r in a if 'body_rotation' in r]
        b=[dict(r,rotation=r['body_rotation']) for r in b if 'body_rotation' in r]
    vals=[relative(x,y) for x in a for y in b]
    out=dict(before_frames=len(a),after_frames=len(b),pairs=len(vals))
    for key in ('twist_deg','total_deg','up_axis_change_deg'):
        out[key+'_p05_median_p95']=np.percentile([v[key] for v in vals],[5,50,95]).tolist() if vals else None
    return out

def video(frames, start, end, name):
    times=np.array([f['camera_read_monotonic'] for f in frames])
    sampletimes=np.arange(start,end,1/8)
    indices=np.searchsorted(times,sampletimes)
    indices=np.clip(indices,0,len(times)-1)
    left=np.maximum(indices-1,0)
    indices=np.where(abs(times[left]-sampletimes)<abs(times[indices]-sampletimes),left,indices)
    temp=ROOT/('forward_hold_video_'+name)
    writer=cv.VideoWriter(temp.name,cv.VideoWriter_fourcc(*'mp4v'),8,(1280,720))
    if not writer.isOpened(): raise RuntimeError('Video writer failed')
    try:
        for idx in indices:
            im=read(F/'camera'/frames[int(idx)]['image'])
            if im is None or im.shape[:2]!=(720,1280): raise RuntimeError('Missing/wrong saved frame')
            writer.write(im)
    finally: writer.release()
    temp.replace(OUT/name)
    cap=cv.VideoCapture(str(OUT/name))
    try:
        count=int(cap.get(cv.CAP_PROP_FRAME_COUNT));fps=cap.get(cv.CAP_PROP_FPS)
        if count!=len(indices) or abs(fps-8)>1e-6: raise RuntimeError('Video verification failed')
    finally: cap.release()
    return dict(path=(OUT/name).relative_to(ROOT).as_posix(),frames=count,fps=fps,seconds=count/fps,wall_clock_start=start,wall_clock_end=end,selection='nearest saved frame at8fps on original camera-read times, no speedup, no motion edits',max_selection_offset_seconds=float(max(abs(times[indices]-sampletimes))),sha256=sha(OUT/name))

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    r=json.loads((F/'result.json').read_bytes())
    frames=json.loads((F/'camera/frames.json').read_bytes())['frames']
    if r['status']!='forward_hold_observation_complete' or not r['all_torque_off_verified'] or not r['settings_restored']:raise RuntimeError('Not a complete/OFF/restored run')
    samples=r['samples'];enabled=r['enabled_at'];phases={}
    for phase in dict.fromkeys(s['phase'] for s in samples):
        ss=[s for s in samples if s['phase']==phase]
        phases[phase]=dict(samples=len(ss),first_t=ss[0]['t'],last_t=ss[-1]['t'])
    hold=[s for s in samples if s['phase']=='stage_8_horizontal_hold']
    if len(hold)!=134 or not all(s['goal']==hold[0]['goal'] for s in hold):raise RuntimeError('Hold goals were not fixed')
    selected=[]
    for phase in ('loaded_countdown','stage_6_grasp_hold','stage_7_horizontal_yaw','stage_8_horizontal_hold','stage_2_fixed_hold','fixed_hold_observation'):
        fs=[f for f in frames if f['phase']==phase]
        selected.extend(fs[::6] if phase=='stage_7_horizontal_yaw' else fs[::2])
    fits=[]
    for n,f in enumerate(selected):
        row=fit(F/'camera'/f['image']);row.update(phase=f['phase'],t=f['camera_read_monotonic']-enabled)
        if row.get('accepted') and 'palm40_corners' in row:
            pp=poses(np.asarray(row['palm40_corners']),.0282)
            if pp:row['body_rotation']=(pp[0]['rotation'].T@np.asarray(row['rotation'])).tolist()
        fits.append(row)
        if n%60==0:print(json.dumps(dict(fitted=n,total=len(selected))),flush=True)
    groups={p:[v for v in fits if v['phase']==p and v.get('accepted')] for p in dict.fromkeys(f['phase'] for f in selected)}
    compare={}
    for name,a,b in (('placement_to_grasp','loaded_countdown','stage_6_grasp_hold'),('placement_to_finish','loaded_countdown','stage_8_horizontal_hold'),('grasp_to_finish','stage_6_grasp_hold','stage_8_horizontal_hold')):
        compare[name]=dict(camera_relative=stats(groups[a],groups[b]),decoded_palm_relative=stats(groups[a],groups[b],True))
    hs=groups['stage_8_horizontal_hold']
    split=(phases['stage_8_horizontal_hold']['first_t']+phases['stage_8_horizontal_hold']['last_t'])/2
    compare['early_to_late_hold']=dict(camera_relative=stats([v for v in hs if v['t']<split],[v for v in hs if v['t']>=split]),decoded_palm_relative=stats([v for v in hs if v['t']<split],[v for v in hs if v['t']>=split],True))
    # Image-plane centers are observables, not registered millimetres.
    marker=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters())
    hold_frames=[f for f in frames if f['phase']=='stage_8_horizontal_hold'];centers=[]
    for f in hold_frames:
        cs,ids,_=marker.detectMarkers(read(F/'camera'/f['image']))
        found={} if ids is None else {int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),cs)}
        if 5 in found and 40 in found:
            centers.append(dict(t=f['camera_read_monotonic']-enabled,top5=found[5].mean(0).tolist(),palm40=found[40].mean(0).tolist()))
    motion={}
    if centers:
        for k in ('top5','palm40'):
            a=np.array([v[k] for v in centers]);motion[k+'_max_from_median_px']=float(max(np.linalg.norm(a-np.median(a,axis=0),axis=1)))
    report=dict(status='saved_forward_and_fixed_hold_image_review',source=F.name,source_result_sha256=sha(F/'result.json'),
        all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
        phase_ranges=phases,hold_goal_samples=len(hold),hold_actual_sample_span_seconds=hold[-1]['t']-hold[0]['t'],hold_all_goals_identical=True,
        minimum_voltage_v=min(min(s['voltage_v']) for s in samples),peak_absolute_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),maximum_temperature_c=max(max(s['temperature_c']) for s in samples),maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        camera=r['camera_recording'],source_frames=len(frames),comparisons=compare,accepted_by_phase={p:len(v) for p,v in groups.items()},
        hold_image_plane_measurements=dict(usable_simultaneous_frames=len(centers),**motion),
        cube_support_confirmed_before_off=r['cube_support_confirmed_before_off'],cube_removal_confirmed=False,cube_removal_after_exit_not_observed=True,
        precise_robot_axis_90_tolerance_verified=False,frame_spread_is_not_absolute_accuracy=True,physical_grip_force_measured=False,left_layer_turn_hold_verified=False,
        source_images_and_log_unchanged=True,devices_opened=False,assistant_motor_writes=0,rows=fits)
    (OUT/'orientation_and_hold_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','camera','phase_ranges')},ensure_ascii=False),flush=True)
    clips=[]
    clips.append(video(frames,frames[0]['camera_read_monotonic'],frames[-1]['camera_read_monotonic'],'full_run.mp4'))
    clips.append(video(frames,enabled+phases['stage_6_grasp_hold']['first_t'],enabled+phases['fixed_hold_observation']['last_t'],'turn_and_hold.mp4'))
    (OUT/'video_exports.json').write_text(json.dumps(clips,indent=2)+'\n',encoding='utf-8')
    contact_sheet(frames,enabled,phases,OUT/'saved_frames_contact_sheet.jpg')
    print('VIDEOS',json.dumps(clips),flush=True)

if __name__=='__main__': main()
