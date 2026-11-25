"""Read installed learning packages and saved arrays without device access."""
from pathlib import Path
import importlib.util
import importlib.metadata
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/prehardware_policy_cycle'


def main():
    versions = {}
    for name in ('torch', 'stable_baselines3', 'gymnasium', 'jax', 'mujoco', 'numpy'):
        versions[name] = None if importlib.util.find_spec(name) is None else importlib.metadata.version(name.replace('_', '-'))
    arrays = {}
    for name in ('right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/candidate.npz',
                 'right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/trajectory.npz'):
        with np.load(ROOT / name, allow_pickle=False) as archive:
            arrays[name] = {key: {'shape': list(archive[key].shape), 'dtype': str(archive[key].dtype)} for key in archive.files}
    report = {'versions': versions, 'saved_arrays': arrays, 'devices_opened': 0}
    if versions['torch']:
        import torch
        report['torch_cuda_available'] = torch.cuda.is_available()
        if torch.cuda.is_available():
            report['torch_cuda_device'] = torch.cuda.get_device_name(0)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'runtime.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
