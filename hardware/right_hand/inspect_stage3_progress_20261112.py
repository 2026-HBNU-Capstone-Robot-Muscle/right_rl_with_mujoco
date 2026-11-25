"""Print only actionable saved progress; never import device or model APIs."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261112/stage3_preparation'

for tag in ('relative_turn_loaded_v4', 'relative_reverse_v5'):
    folder = OUT / tag
    if (folder / 'review.json').exists():
        r = json.loads((folder / 'review.json').read_bytes())
        print(json.dumps({'tag': tag, 'steps': r['steps'], 'stop': r['stop'],
              'best_error_deg': r['best_sampled_error_deg'], 'final_hold': r['final_hold'],
              'passed': r['conditional_bridge_and_hold_passed'], 'prediction_rejections': r['prediction_rejections'],
              'worst': r['worst'], 'final': {k: r['final'][k] for k in ('error_deg', 'tilt_deg', 'contact_finger_normals_N', 'tracking_rad')}}))
    elif (folder / 'progress.json').exists():
        r = json.loads((folder / 'progress.json').read_bytes()); ev = r['actual']
        print(json.dumps({'tag': tag, 'step': r['step'], 'error_deg': ev['error_deg'],
                          'center_error_mm': ev['center_target_error_mm'], 'elapsed_s': r['elapsed_seconds']}))
