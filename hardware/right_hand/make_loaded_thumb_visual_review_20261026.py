"""Create a timestamped index and contact sheet of original saved images."""
import hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
if len(sys.argv)>1:SOURCE=(ROOT/sys.argv[1]).resolve()
if not SOURCE.is_relative_to(ROOT):raise ValueError('Saved source must be inside outputs')
OUT=SOURCE/'visual_review'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
    frame=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    if frame is None:raise ValueError('Unreadable saved image: '+str(p))
    return frame
def main():
    OUT.mkdir(exist_ok=True);frames=json.loads((SOURCE/'camera/frames.json').read_bytes())['frames']
    first=next(i for i,v in enumerate(frames) if v['phase']=='loaded_countdown')
    last=max(i for i,v in enumerate(frames) if v['phase']=='loaded_turn_observation')
    clip=frames[first:last+1];start=clip[0]['camera_read_monotonic'];stamps=np.asarray([v['camera_read_monotonic'] for v in clip])
    assert np.all(np.diff(stamps)>0) and all(v['image_signal_usable'] for v in clip)
    selection=[clip[0]]
    for phase in dict.fromkeys(v['phase'] for v in clip):
        group=[v for v in clip if v['phase']==phase]
        if phase=='stage_6_loaded_quarter_reference':indices=np.linspace(0,len(group)-1,6).round().astype(int)
        elif phase in ('stage_9_loaded_thumb_retreat','stage_10_loaded_index_middle_lift'):indices=np.asarray([len(group)//2,len(group)-1])
        else:indices=np.asarray([len(group)-1])
        for i in indices:
            if group[i] not in selection:selection.append(group[i])
    width,height,label=448,252,48;cols=4;rows=(len(selection)+cols-1)//cols
    sheet=np.full((rows*(height+label),cols*width,3),235,dtype=np.uint8)
    for i,v in enumerate(selection):
        x=(i%cols)*width;y=(i//cols)*(height+label)
        frame=load(SOURCE/'camera'/v['image']);sheet[y+label:y+label+height,x:x+width]=cv2.resize(frame,(width,height),interpolation=cv2.INTER_AREA)
        phase=v['phase'].replace('stage_','').replace('loaded_','').replace('_',' ')
        cv2.putText(sheet,f"{v['camera_read_monotonic']-start:5.1f}s | {v['image']}",(x+8,y+17),cv2.FONT_HERSHEY_SIMPLEX,.43,(20,20,20),1,cv2.LINE_AA)
        cv2.putText(sheet,phase,(x+8,y+37),cv2.FONT_HERSHEY_SIMPLEX,.42,(20,20,20),1,cv2.LINE_AA)
    ok,encoded=cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,94]);assert ok
    (OUT/'actual_rotation_contact_sheet.jpg').write_bytes(encoded.tobytes())
    report=dict(source=SOURCE.name,result_sha256=sha(SOURCE/'result.json'),camera_metadata_sha256=sha(SOURCE/'camera/frames.json'),
        frames=clip,selected_frames=selection,source_start_monotonic=float(stamps[0]),source_end_monotonic=float(stamps[-1]),
        recorded_span_seconds=float(stamps[-1]-stamps[0]),frame_count=len(clip),maximum_recorded_frame_gap_seconds=float(np.max(np.diff(stamps))),
        all_saved_images_usable=True,frames_are_saved_camera_stills=True,images_content_unchanged=True,
        source_images_perceptually_reviewed_all=False,precise_3D_angle_verified=False,hardware_commands_sent=0)
    (OUT/'video_source_index.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('frames','selected_frames')},indent=2),flush=True)
if __name__=='__main__':main()
