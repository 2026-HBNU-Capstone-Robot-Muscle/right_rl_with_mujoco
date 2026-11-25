"""Read compact status from the latest saved anchor attempts, without device I/O."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for path in sorted(ROOT.glob('leap_sequence_flip_20261006_23*.json')):
    doc = json.loads(path.read_text(encoding='utf-8'))
    print(path.name)
    for key in ('timestamp', 'status', 'phase', 'all_torque_off_verified',
                'settings_restored', 'error', 'stop_reason', 'stop_requested',
                'anchor_rigid_projection_error_px', 'capture_anchor_placement'):
        if key in doc:
            print(f'  {key}: {doc[key]}')
    if 'vision_recording' in doc:
        v = doc['vision_recording']
        print(f'  vision folder: {v.get("folder")}')
        print(f'  vision error: {v.get("error")}')
