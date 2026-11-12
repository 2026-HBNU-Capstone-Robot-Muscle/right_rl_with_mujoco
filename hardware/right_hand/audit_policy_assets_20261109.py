"""Inventory saved policy and training evidence without device access."""
from pathlib import Path
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    archive = ROOT.parent / 'work/Leap_Test/policy/policy_np.npz'
    with np.load(archive, allow_pickle=False) as z:
        arrays = {name: {'shape': list(z[name].shape), 'dtype': str(z[name].dtype),
                         'finite': bool(np.isfinite(z[name]).all()) if z[name].dtype.kind in 'fi' else None}
                  for name in z.files}
    report = {'original_policy': {'path': str(archive), 'sha256': sha(archive), 'arrays': arrays},
              'training_records': [], 'root_evaluations': [], 'devices_opened': 0}
    for folder in sorted(ROOT.iterdir()):
        if not folder.is_dir() or not folder.name.startswith('sac_'):
            continue
        record = {'folder': folder.name, 'records': {}}
        for name in ('result.json', 'pilot_result.json', 'settings.json', 'numpy_export/manifest.json',
                     'numpy_export/windows_parity.json', 'evaluation.json', 'eval_result.json'):
            path = folder / name
            if path.is_file():
                record['records'][name] = json.loads(path.read_bytes())
        if record['records']:
            report['training_records'].append(record)
    for name in ('sac_supported_teacher_batch_eval.json', 'sac_stage0_teacher_v2_result.json',
                 'sac_oct8_recompiled_policy_probe_result.json'):
        path = ROOT / name
        if path.is_file():
            data = json.loads(path.read_bytes())
            small = {k: v for k, v in data.items() if not isinstance(v, (list, dict))}
            report['root_evaluations'].append({'path': name, 'summary': small})
    (OUT / 'asset_inventory.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(report['original_policy'], ensure_ascii=False))
    for item in report['training_records']:
        results = {name: value for name, value in item['records'].items() if name.endswith('result.json')}
        settings = item['records'].get('settings.json', {})
        export = item['records'].get('numpy_export/manifest.json', {})
        print(json.dumps({'folder': item['folder'], 'results': results,
                          'settings': {k: settings[k] for k in ('timesteps', 'residual', 'reference_residual', 'stage0_forward', 'teacher_sha256') if k in settings},
                          'numpy_export': export}, ensure_ascii=False))
    print(json.dumps({'root_evaluations': report['root_evaluations']}))


if __name__ == '__main__':
    main()
