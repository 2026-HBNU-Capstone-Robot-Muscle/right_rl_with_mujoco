"""Saved support image/telemetry review only, no device interfaces."""
import hashlib,json
from pathlib import Path
import cv2 as cv,numpy as np
ROOT=Path(__file__).resolve().parent;SOURCE=ROOT/'quarter_support_photo_20261025_221703_805528'
OUT=ROOT/'right_hand_rotation_20261024/support_photo_review_221703';OUT.mkdir(exist_ok=True)
r=json.loads((SOURCE/'result.json').read_bytes());samples=r['samples']
result={k:r.get(k) for k in ('status','all_torque_off_verified','settings_restored','cube_removal_confirmed','loaded_goal_count','stopped_before_loaded_motion','placement_position_ticks','placement_goal_ticks')}
result.update(source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256((SOURCE/'result.json').read_bytes()).hexdigest(),
    max_tracking_rad=max(max(s['tracking_rad']) for s in samples),peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
    min_voltage_v=min(min(s['voltage_v']) for s in samples),max_temperature_c=max(max(s['temperature_c']) for s in samples),
    camera=r['camera_recording'],camera_off_adjustment_waits=r.get('camera_off_adjustment_waits',[]),
    motor_write_events=len(r['write_events']),placement_error_ticks=(np.asarray(r['placement_position_ticks'])-r['placement_goal_ticks']).tolist(),
    preparation_and_support_photo_completed=True,physical_rotation_verified=False,hardware_commands_sent_by_this_review=0)
dictionary=cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50);params=cv.aruco.DetectorParameters();params.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX;det=cv.aruco.ArucoDetector(dictionary,params)
for name in ('empty_support_reference','cube_support_side'):
    p=SOURCE/(name+'.jpg');im=cv.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv.IMREAD_COLOR)
    cs,ids,_=det.detectMarkers(im);result[name+'_markers']={str(int(i)):c.reshape(4,2).tolist() for i,c in zip(ids.ravel(),cs)} if ids is not None else {}
    crop=im[400:650,390:900];cv.imencode('.png',cv.resize(crop,None,fx=2,fy=2,interpolation=cv.INTER_CUBIC))[1].tofile(str(OUT/(name+'_support_crop.png')))
frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
result['phase_frame_counts']={phase:sum(f['phase']==phase for f in frames) for phase in sorted(set(f['phase'] for f in frames))}
placed=[f for f in frames if f['phase']=='place_cube']
selected=[placed[round(k*(len(placed)-1)/5)] for k in range(6)] if placed else []
contact=np.zeros((2*290,3*510,3),np.uint8)
for k,f in enumerate(selected):
    im=cv.imdecode(np.frombuffer((SOURCE/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR);tile=cv.resize(im[380:650,360:930],(510,270));y,x=divmod(k,3);contact[y*290:y*290+270,x*510:(x+1)*510]=tile
    cv.putText(contact,f['image'],(x*510+5,y*290+286),cv.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
cv.imencode('.jpg',contact)[1].tofile(str(OUT/'placement_support_sequence.jpg'))
(OUT/'initial_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
