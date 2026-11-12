"""Saved-file evidence for completed loaded grasp/open, without device access."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from relative_marker_rotation_20261026 import detect_markers,compare_marker_frames
ROOT=Path(__file__).resolve().parent;FOLDER=ROOT/'new_pair_grasp_20261027_050946_894527'
def imread(p):return cv2.imdecode(np.fromfile(p,np.uint8),cv2.IMREAD_COLOR)
def main():
    r=json.loads((FOLDER/'result.json').read_bytes());assert r['status']=='new_pair_grasp_observation_complete' and r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed'];samples=r['samples'];out=FOLDER/'visual_review';out.mkdir(exist_ok=True);phases={}
    for phase in dict.fromkeys(s['phase'] for s in samples):
        rows=[s for s in samples if s['phase']==phase];phases[phase]=dict(samples=len(rows),time_first=rows[0]['t'],time_last=rows[-1]['t'],min_voltage_v=min(min(s['voltage_v']) for s in rows),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in rows),max_tracking_rad=max(max(s['tracking_rad']) for s in rows),median_actual_ticks=np.median([s['actual'] for s in rows],axis=0).tolist(),median_current_ma=np.median([s['current_ma'] for s in rows],axis=0).tolist())
    empty=json.loads((ROOT/'fresh_empty_grasp_20261027_032949_661470/result.json').read_bytes());erows=[s for s in empty['samples'] if s['phase']=='stage_5_empty_hold'];lrows=[s for s in samples if s['phase']=='stage_6_grasp_hold'];comparison=dict(empty_goal=erows[-1]['goal'],loaded_goal=lrows[-1]['goal'],same_goals=erows[-1]['goal']==lrows[-1]['goal'],empty_median_current_ma=np.median([s['current_ma'] for s in erows],axis=0).tolist(),loaded_median_current_ma=np.median([s['current_ma'] for s in lrows],axis=0).tolist(),empty_median_actual_ticks=np.median([s['actual'] for s in erows],axis=0).tolist(),loaded_median_actual_ticks=np.median([s['actual'] for s in lrows],axis=0).tolist())
    cal=np.load(ROOT/'camera1_720.npz');K,dist=cal['K'],cal['dist'];detected={name:detect_markers(imread(FOLDER/name)) for name in ('cube_before_grasp.jpg','cube_grasp_hold.jpg','cube_after_open.jpg')};relative=[]
    for name in ('cube_grasp_hold.jpg','cube_after_open.jpg'):
        a=detected['cube_before_grasp.jpg'];b=detected[name];shared=sorted((set(a)&set(b))-{40,41})
        for ref in (40,41):
            if ref in a and ref in b and len(shared)>=2:
                top=5 if 5 in shared else None;v=compare_marker_frames(a,b,ref,shared,K,dist,top_marker_id=top,up_from_cross=tuple(shared[:2]) if top is None else None,max_reference_pose_change_deg=15);relative.append(dict(before='cube_before_grasp.jpg',after=name,**v))
    manifest=json.loads((FOLDER/'camera/frames.json').read_bytes());frames=manifest['frames'];chosen=[]
    for phase,fraction in [('loaded_countdown',.5),('stage_5_loaded_grasp',.15),('stage_5_loaded_grasp',.5),('stage_5_loaded_grasp',.85),('stage_6_grasp_hold',.5),('stage_7_loaded_open',.25),('stage_7_loaded_open',.6),('stage_7_loaded_open',.9),('opened_before_remove',.5)]:
        candidates=[f for f in frames if f['phase']==phase]
        if candidates:chosen.append(candidates[min(len(candidates)-1,int((len(candidates)-1)*fraction))])
    canvas=np.full((900,1440,3),255,np.uint8)
    for i,f in enumerate(chosen):
        x=i%3*480;y=i//3*300;canvas[y+30:y+300,x:x+480]=cv2.resize(imread(FOLDER/'camera'/f['image']),(480,270));cv2.putText(canvas,f['phase'].replace('stage_','')[:37],(x+6,y+20),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,0,0),1)
    cv2.imencode('.jpg',canvas)[1].tofile(out/'grasp_open_contact_sheet.jpg')
    start=r['placement_photo']['camera_read_monotonic']-.4;end=r['opened_before_remove_photo']['camera_read_monotonic']+.6;video_frames=[f for f in frames if start-.5<=f['camera_read_monotonic']<=end+.5];path=out/'actual_grasp_open_realtime.mp4';writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,720));assert writer.isOpened();index=0
    for stamp in np.arange(start,end,.1):
        while index+1<len(video_frames) and video_frames[index+1]['camera_read_monotonic']<=stamp:index+=1
        writer.write(imread(FOLDER/'camera'/video_frames[index]['image']))
    writer.release()
    summary=dict(status='loaded_grasp_open_recording_completed',source=(FOLDER/'result.json').relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256((FOLDER/'result.json').read_bytes()).hexdigest(),plan_version=r['plan_version'],health_samples=len(samples),duration_from_enable_s=max(s['t'] for s in samples),minimum_voltage_v=min(min(s['voltage_v']) for s in samples),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),voltage_events=r.get('voltage_events',[]),camera_frames=manifest['image_signal_usable_frames'],blank_frames=manifest['image_signal_blank_frames'],phases=phases,loaded_vs_empty=comparison,detected_photo_markers={name:sorted(v) for name,v in detected.items()},relative_marker_results=relative,all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,physical_lift_grasp_force_not_verified=True,rotation_was_not_commanded=True,physical_horizontal_90_verified=False,forward_flip_successes_retained=2,video=(out/'actual_grasp_open_realtime.mp4').relative_to(ROOT).as_posix(),assistant_motor_commands_sent=0)
    (out/'review.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in summary.items() if k not in ('phases','loaded_vs_empty','relative_marker_results')},indent=2));print('CURRENT_COMPARISON',json.dumps(comparison));print('RELATIVE',json.dumps(relative))
if __name__=='__main__':main()
