"""Read-only summary of saved support-camera evidence; no motor/camera access."""
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'sequence_vision_20261008_184238_359328'
frames = json.loads((FOLDER / 'result.json').read_text(encoding='utf-8'))['frames']
report = json.loads((ROOT / 'leap_support_capture_result.json').read_text(encoding='utf-8'))
by_phase = {}
for phase in dict.fromkeys(frame['phase'] for frame in frames):
    rows = [f for f in frames if f['phase'] == phase]
    counts = Counter(tuple(sorted(f.get('detected_ids', []))) for f in rows)
    by_phase[phase] = {
        'frames': len(rows),
        'detected_ids': {','.join(map(str, ids)) or 'none': count
                         for ids, count in sorted(counts.items())},
        'hand_plus_two_cube_frames': sum(40 in f.get('detected_ids', []) and
                                         sum(0 <= i < 6 for i in f.get('detected_ids', [])) >= 2
                                         for f in rows),
    }
out = {
    'folder': str(FOLDER),
    'status': report['status'],
    'all_torque_off_verified': report['all_torque_off_verified'],
    'settings_restored': report['settings_restored'],
    'policy_used': report['policy_used'],
    'placement_snapshot': report.get('cube_placement_camera_snapshot'),
    'phases': by_phase,
    'interpretation': 'Camera phase labels and raw images must be reviewed together; no inferred cube contact point.',
}
(ROOT / 'sac_latest_support_frame_audit_result.json').write_text(
    json.dumps(out, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(out, ensure_ascii=False, indent=2))
