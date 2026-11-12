import json
from pathlib import Path

root = Path(__file__).resolve().parent
for folder in sorted(root.glob('raised_support_capture_20261104_*')):
    path = folder / 'result.json'
    if not path.exists():
        continue
    data = json.loads(path.read_bytes())
    print(folder.name)
    for key in ('post_placement_motion', 'placement_goal_ticks', 'placement_position_ticks',
                'passive_observation_policy', 'cube_removal_confirmation_source',
                'physical_grasp_verified', 'physical_rotation_verified',
                'final_off_reference_valid', 'final_off_endpoint_drift_ticks'):
        print(key, json.dumps(data.get(key), ensure_ascii=False))
    samples = data.get('samples', [])
    print('sample_first', json.dumps(samples[0] if samples else None, ensure_ascii=False)[:6500])
    hold = [s for s in samples if s.get('phase') == 'stage_5_loaded_grasp']
    print('hold_first', json.dumps(hold[0] if hold else None, ensure_ascii=False)[:6500])
