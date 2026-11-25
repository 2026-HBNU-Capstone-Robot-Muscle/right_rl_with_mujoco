"""Motor-free check of scrambled-cube-compatible center-marker pose fitting."""

import json

from sac_center_spin_pose import ROOT, fit_image


IMAGES = [
    'leap_two_face_preview_20261009_223918_666289.png',
    'leap_two_face_preview_20261009_225422_808088.png',
    'vision_check_20261004_192532_747526/eligible.png',
    'vision_sweep_20261006_011011_994373/00-10s_eligible.png',
    'vision_sweep_20261006_011011_994373/10-20s_eligible.png',
    'vision_sweep_20261006_011011_994373/40-50s_eligible.png',
    'vision_sweep_20261006_011011_994373/60-70s_eligible.png',
]


def main():
    rows = []
    for name in IMAGES:
        row = {'image': name}
        try:
            result = fit_image(ROOT / name)
            row.update(status='ok', visible_ids=result['visible_ids'],
                       spins=result['best']['spins'],
                       reprojection_px=result['best']['mean_reprojection_px'],
                       ambiguity_gap_px=result['ambiguity_gap_px'])
        except (ValueError, RuntimeError) as exc:
            row.update(status='insufficient', reason=str(exc))
        rows.append(row)
    report = {'status': 'offline_center_spin_benchmark', 'motor_commands_sent': 0,
              'rows': rows,
              'caveat': 'Two visible centers only; single-center and full-occlusion periods remain unsolved.'}
    (ROOT / 'sac_center_spin_archive_benchmark_result.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
