"""Review saved frames only. No device API or production runtime changes."""
import contextlib,io,json
from pathlib import Path
import cv2 as cv,numpy as np
with contextlib.redirect_stdout(io.StringIO()):import measure_loaded_green_grid_20261025 as grid
ROOT=Path(__file__).resolve().parent;SOURCE=grid.SOURCE;OUT=grid.OUT
saved=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
frames=[f for f in saved if f['phase'] in ('stage_6_loaded_quarter_turn','stage_7_loaded_observation','stage_2_fixed_hold')]
start=frames[0]['camera_read_monotonic'];reference=grid.solve(grid.obj0,grid.img0,grid.K)[0][1]
rows=[];previous=reference
for n,f in enumerate(frames):
    try:
        im,obj,img,_=grid.corners_and_grid(SOURCE/'camera'/f['image']);solutions=grid.solve(obj,img,grid.K)
        distances=[float(np.linalg.norm(cv.Rodrigues(R@previous.T)[0])*180/np.pi) for err,R,t in solutions]
        best=min(range(len(solutions)),key=lambda j:solutions[j][0]+.02*distances[j])
        error,R,t=solutions[best]
        if error>4:raise ValueError('Large grid reprojection residual')
        previous=R;angle=float(np.linalg.norm(cv.Rodrigues(R@reference.T)[0])*180/np.pi)
        rows.append(dict(image=f['image'],t=f['camera_read_monotonic']-start,phase=f['phase'],rotation_deg=angle,
            grid_error_px=error,other_branch_error_px=solutions[1-best][0],grid_points=len(obj),R=R.tolist()))
    except (ValueError,RuntimeError,StopIteration,TypeError,KeyError,AttributeError,cv.error):
        rows.append(dict(image=f['image'],t=f['camera_read_monotonic']-start,phase=f['phase'],unusable_grid=True))
valid=[r for r in rows if 'rotation_deg' in r];windows=[]
for a,b in ((0,5),(5,10),(10,15),(15,20),(20,25),(25,30),(30,35),(35,40),(40,46)):
    sample=[r['rotation_deg'] for r in valid if a<=r['t']<b]
    if sample:windows.append(dict(start_s=a,end_s=b,count=len(sample),median_provisional_change_deg=float(np.median(sample)),min_deg=min(sample),max_deg=max(sample)))
report=dict(frames=len(rows),usable=len(valid),windows=windows,rows=rows,
    calibration_camera_identity_reverified=False,camera_motion_compensated=False,diagnostic_not_certified_angles=True,
    physical_quarter_turn_verified=False,user_observation='Approximately45deg tilt in groove, not flipped; uncertain recollection')
(OUT/'loaded_video_timeline.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
canvas=np.full((3*250,4*360,3),235,np.uint8)
for j,t in enumerate(np.linspace(0,frames[-1]['camera_read_monotonic']-start,12)):
    f=min(frames,key=lambda f:abs(f['camera_read_monotonic']-start-t));im=cv.imdecode(np.frombuffer((SOURCE/'camera'/f['image']).read_bytes(),np.uint8),cv.IMREAD_COLOR)
    crop=cv.resize(im[0:430,260:820],(360,220));r,c=divmod(j,4);canvas[r*250:r*250+220,c*360:(c+1)*360]=crop
    cv.putText(canvas,f'{f["camera_read_monotonic"]-start:.1f}s',(c*360+10,r*250+241),cv.FONT_HERSHEY_SIMPLEX,.55,(20,20,20),1)
cv.imencode('.jpg',canvas)[1].tofile(str(OUT/'loaded_turn_contact_sheet.jpg'))
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
