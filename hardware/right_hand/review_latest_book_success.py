"""Review saved preparation runs without opening any device."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
import cv2

ROOT=Path(__file__).resolve().parent
records=[]
for name in ('book_prepare_empty_20261024_165322_592515','book_prepare_empty_20261024_165621_852486'):
    folder=ROOT/name
    result=json.loads((folder/'result.json').read_bytes())
    samples=result.get('samples',[])
    row={k:result.get(k) for k in ('status','error','all_torque_off_verified','settings_restored','current_start_screen_accepted','enabled_at','camera_recording')}
    row.update(folder=name,sample_count=len(samples))
    row['write_event_count']=len(result.get('write_events',[]))
    row['first_write_event']=next(iter(result.get('write_events',[])),None)
    if samples:
        peak=max((max(s['tracking_rad']),i,s) for i,s in enumerate(samples))
        row.update(phase_counts=dict(Counter(s['phase'] for s in samples)),last_sample_time_s=samples[-1]['t'],
            minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
            peak_absolute_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
            maximum_tracking_rad=peak[0],peak_tracking_phase=peak[2]['phase'],
            peak_tracking_motor_id=1+int(np.argmax(peak[2]['tracking_rad'])),
            maximum_read_ms=max(s['read_ms'] for s in samples),
            last_goal_ticks=samples[-1]['goal'],last_measured_ticks=samples[-1]['actual'],
            final_off_reference_valid=result.get('final_off_reference_valid'),
            on_to_off_position_change_ticks=result.get('on_to_off_position_change_ticks'))
        frames=json.loads((folder/'camera/frames.json').read_bytes())['frames']
        enabled=result['enabled_at'];selected=[]
        for label,target in [('start',0),('roots lifted',14),('thumb clearance',16),('preparation',30),('final ON',43.9),('after OFF',46)]:
            frame=min(frames,key=lambda f:abs(f['camera_read_monotonic']-enabled-target))
            selected.append(dict(label=label,seconds_from_enable=frame['camera_read_monotonic']-enabled,**frame))
        row['selected_frames']=selected
        tiles=[]
        for f in selected:
            im=cv2.imdecode(np.frombuffer((folder/'camera'/f['image']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            im=cv2.resize(im,(640,360));tile=np.zeros((395,640,3),np.uint8);tile[35:]=im
            cv2.putText(tile,f"{f['label']}  t={f['seconds_from_enable']:.1f}s",(10,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
            tiles.append(tile)
        sheet=np.vstack([np.hstack(tiles[:3]),np.hstack(tiles[3:])])
        sheet_path=ROOT/'right_hand_rotation_20261024/book20_current_review/preparation_actual_frames.jpg'
        cv2.imencode('.jpg',sheet)[1].tofile(str(sheet_path))
        row['contact_sheet']=str(sheet_path)
    records.append(row)
out=ROOT/'right_hand_rotation_20261024/book20_current_review/latest_hardware_review.json'
out.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
for row in records: print(json.dumps(row,ensure_ascii=False))
