"""Review the actual new-start placement without opening a device."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'reverse_start_placement_20261025_075405_164917/result.json'
OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/actual_075405_placement_review'

def main():
    r=json.loads(SOURCE.read_bytes());samples=r.get('samples',[])
    fields=('status','error','prepared_empty_photo','prepared_empty_ticks','placement_photo','placement_position_ticks','placement_goal_ticks',
        'placement_current_ma','placed_hold_photo','placed_hold_state','cube_removal_confirmed','all_torque_off_verified','settings_restored',
        'final_on_state','final_command','final_off_reference_valid','final_off_snapshots','on_to_off_position_change_ticks','camera_recording')
    report={k:r.get(k) for k in fields}
    report.update(source=str(SOURCE.relative_to(ROOT)),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        max_tracking_rad=max(max(s['tracking_rad']) for s in samples),min_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
        max_temperature_c=max(max(s['temperature_c']) for s in samples),physical_cube_rotation_tested=False,review_device_writes=0)
    OUT.mkdir(exist_ok=True);(OUT/'review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('placed_hold_state','final_on_state','final_off_snapshots','camera_recording')},ensure_ascii=False))
    print('NAMED_IMAGES',[str(p) for p in SOURCE.parent.rglob('*.jpg') if not p.name.startswith('frame_')])
    print('RESULT_KEYS',list(r))

if __name__=='__main__':main()
