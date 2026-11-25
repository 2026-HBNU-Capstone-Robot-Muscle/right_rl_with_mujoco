"""Review the saved EMPTY start trial without opening any device."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'reverse_start_empty_20261025_043035_076434'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_043035_review'

def main():
    OUT.mkdir(exist_ok=True);blob=(SOURCE/'result.json').read_bytes();r=json.loads(blob)
    samples=r.get('samples',[])
    phases={}
    for row in samples:phases.setdefault(row['phase'],[]).append(row)
    summaries=[]
    for phase,rows in phases.items():
        q=rows[-1]
        summaries.append(dict(phase=phase,samples=len(rows),last_actual=q.get('actual'),last_goal=q.get('goal'),
            max_tracking_rad=max([max(s.get('tracking_rad',[0])) for s in rows]),
            min_voltage_v=min(min(s['voltage_v']) for s in rows),
            peak_current_ma=max([max(abs(v) for v in s.get('current_ma',[0])) for s in rows])))
    report=dict(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(blob).hexdigest(),
        status=r['status'],all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),
        final_on_state=r.get('final_on_state'),final_command=r.get('final_command'),
        final_off_snapshots=r.get('final_off_snapshots'),on_to_off_position_change_ticks=r.get('on_to_off_position_change_ticks'),
        camera_recording=r.get('camera_recording'),phases=summaries,physical_cube_rotation_tested=False,
        review_motor_writes=0)
    (OUT/'execution_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(dict(status=r['status'],max_tracking_rad=max(max(s['tracking_rad']) for s in samples),
        peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        min_voltage_v=min(min(s['voltage_v']) for s in samples),
        max_temperature_c=max(max(s['temperature_c']) for s in samples),
        final_goal_error_ticks=(np.array(r['final_on_state']['position'])-np.array(r['final_command'])).tolist(),
        on_to_off_position_change_ticks=r.get('on_to_off_position_change_ticks'),
        camera=r.get('camera_recording')),ensure_ascii=False))
    frames=json.loads((SOURCE/'camera/frames.json').read_bytes())
    print('TIMING',r['enabled_at'],samples[0]['t'],samples[-1]['t'])
    print('FRAME_META',str(frames)[:600],str(frames)[-600:])

if __name__=='__main__':main()
