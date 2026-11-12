"""Offline policy provenance/parity audit, distinct from dynamic task success."""
from pathlib import Path
import hashlib
import importlib.util
import json
import numpy as np

from sac_reference_numpy import ReferencePolicy, check

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'right_hand_rotation_20261109/policy_audit'
WORK = ROOT.parent / 'work/Leap_Test'
MIDDLEWARE = ROOT / 'team_integration_20261109/middleware_review/source'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    item = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(item)
    return item


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    policy_path = WORK / 'policy/policy_np.npz'
    original = module('offline_original_policy', WORK / 'deploy/policy_np.py').Policy(policy_path)
    adapter_path = MIDDLEWARE / 'src/control/leap_hand_control/leap_hand_control/policy_adapter.py'
    adapter = module('offline_middleware_policy', adapter_path).LeapPolicyAdapter(policy_path)
    old_obs = module('offline_original_observation', WORK / 'deploy/obs_builder.py').ObsBuilder()
    new_obs = module('offline_middleware_observation', adapter_path.parent / 'observation_builder.py').LeapObservationBuilder()
    rng = np.random.default_rng(20261109)
    observations = original.mean + original.std * rng.normal(size=(512, 108))
    expected = np.stack([original(obs) for obs in observations])
    actual = np.stack([adapter.infer(obs) for obs in observations])
    actor_error = float(np.max(np.abs(actual - expected)))
    obs_error = 0.
    for index in range(128):
        q = rng.uniform(-.1, 1.5, 16)
        command = q + rng.uniform(-.02, .02, 16)
        palm = rng.normal(0, .05, 3)
        center = rng.normal(0, .05, 3)
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        goal = rng.normal(size=4)
        goal /= np.linalg.norm(goal)
        if index % 11 == 0:
            old_obs.fill_ori_history(quat, goal)
            new_obs.fill_orientation_history(quat, goal)
        a = old_obs.build(q, command, palm, center, quat, goal)
        b = new_obs.build(q, command, palm, center, quat, goal)
        obs_error = max(obs_error, float(np.max(np.abs(a - b))))
        action = rng.uniform(-1, 1, 16)
        old_obs.set_last_act(action)
        new_obs.set_last_action(action)
    reference = ReferencePolicy()
    parity = check()
    reference_folder = ROOT / 'sac_forward_support_reference_v1_20k'
    checkpoint = reference_folder / 'checkpoints' / reference.manifest['checkpoint']
    checkpoint_rows = []
    for relative, expected_hash in reference.manifest['checkpoint_files_sha256'].items():
        path = checkpoint / relative
        checkpoint_rows.append({'path': relative, 'present': path.is_file(),
                                'hash_matches': path.is_file() and sha(path) == expected_hash})
    evaluations = []
    for name in ('supported_batch_eval.json', 'supported_batch_eval_sac_cube55_resets_cube55mm.json',
                 'supported_batch_eval_sac_start_tolerance_resets.json',
                 'supported_batch_eval_sac_cube55_resets_cube55mm_positionbias15mm.json'):
        path = reference_folder / name
        if not path.is_file():
            continue
        data = json.loads(path.read_bytes())
        scalars = {k: v for k, v in data.items() if not isinstance(v, (list, dict))}
        evaluations.append({'path': str(path.relative_to(ROOT)), 'sha256': sha(path), 'summary': scalars})
    report = {
        'status': 'saved_policy_contract_audit_complete',
        'original_policy_sha256': sha(policy_path),
        'original_108_policy_matches_team_recorded_sha256': sha(policy_path) == 'dc220d146bc5532c4ae09907c6d6bbc887335389d215afaafb69eeab679560e7',
        'original_vs_middleware_inference': {'samples': 512, 'maximum_action_difference': actor_error},
        'original_vs_middleware_observations': {'consecutive_samples': 128, 'maximum_difference': obs_error},
        'trained_reference_residual': {'observation_size': 110, 'action_size': 16,
                                       'training_timesteps_recorded': 20000,
                                       'control_dt_s': .05,
                                       'reference_plus_sac_residual_rad': .05,
                                       'actor_sha256': reference.manifest['actor_sha256'],
                                       'saved_independent_jax_fixture_comparison': parity,
                                       'checkpoint_file_checks': checkpoint_rows},
        'historical_dynamic_evaluations_not_rerun_today': evaluations,
        'current_rig_dynamic_policy_evaluation_completed': False,
        'new_training_performed_today': False,
        'physical_policy_execution_verified': False,
        'devices_opened': 0,
    }
    (OUT / 'contract_audit.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))
    if actor_error > 1e-12 or obs_error > 1e-12 or not all(row['hash_matches'] for row in checkpoint_rows):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
