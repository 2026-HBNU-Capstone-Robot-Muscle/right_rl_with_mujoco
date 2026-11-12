"""Read latest user-pasted terminal log and saved sequence diagnostics."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ATTACHMENT = Path(r'C:\Users\김준서\.codex\attachments\2fb7646a-eb64-4001-ad55-658e6f1283cd\Pasted text.txt')


def main():
    try:
        print('PASTED LOG:\n' + ATTACHMENT.read_text(encoding='utf-8-sig'))
    except Exception as exc:
        print(f'PASTED LOG UNAVAILABLE: {exc}')
    for path in sorted(ROOT.glob('leap_sequence_flip_20261011_*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        vision = data.get('vision_recording') or {}
        samples = data.get('samples') or []
        print(json.dumps(dict(file=path.name,status=data.get('status'),
                              phase=data.get('failed_phase') or data.get('phase'),
                              error=data.get('error'),
                              all_torque_off_verified=data.get('all_torque_off_verified'),
                              sample_count=len(samples),
                              stage_phases=sorted({s['phase'] for s in samples if s['phase'].startswith('stage_')}),
                              placement=data.get('cube_placement_camera_snapshot'),
                              vision_folder=vision.get('folder')),
                         ensure_ascii=False))
        result = vision.get('result')
        if result and Path(result).is_file():
            vis = json.loads(Path(result).read_text(encoding='utf-8'))
            print(json.dumps(dict(vision=Path(result).parent.name,
                                  phase_ends=vis.get('phase_end_images'),
                                  by_phase={k: dict(frames=v.get('frames'),
                                                    eligible_percent=v.get('eligible_percent'),
                                                    by_cube_ids=v.get('by_cube_ids'))
                                            for k, v in vis.get('by_phase', {}).items()}),
                             ensure_ascii=False))


if __name__ == '__main__':
    main()
