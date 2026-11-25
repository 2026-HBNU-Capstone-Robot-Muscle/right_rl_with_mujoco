"""Bounded summaries from saved offline reports; no devices."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic'
for path in sorted(FOLDER.glob('*/review.json')):
    r=json.loads(path.read_bytes())
    if 'cases' not in r:continue
    cases=[]
    for row in r['cases']:
        checkpoints=row['phase_checkpoints'];held=checkpoints.get('yaw_hold',{});grasp=checkpoints.get('grasp_hold',{})
        cases.append(dict(seed=row['seed'],stop=row['stop'],last_phase=row['last_phase'],samples=row['samples_completed'],
            held_twist=held.get('horizontal_twist_deg'),held_red_up_error=held.get('red_up_error_deg'),held_green_up_error=held.get('green_up_error_deg'),
            grasp_twist=grasp.get('horizontal_twist_deg'),grasp_red_up_error=grasp.get('red_up_error_deg'),
            max_overlap_mm=row['max_cube_overlap_mm'],max_tracking_rad=row['max_tracking_rad']))
    print(json.dumps(dict(folder=path.parent.name,cases=cases),ensure_ascii=False))
