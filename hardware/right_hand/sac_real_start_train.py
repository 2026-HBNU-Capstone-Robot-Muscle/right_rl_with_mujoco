"""Offline SAC training from the measured real-hand start distribution.

This does not export a hardware policy. Camera occlusion and sim-to-real
validation remain separate gates.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import time

os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

from brax.training.agents.sac import train as sac
from mujoco_playground import registry

from sac_real_start_env import RealStartFaceUp
from sac_stage2_start_env import Stage2StartFaceUp
from sac_stage0_start_env import Stage0StartFaceUp
from sac_forward_geometry import METRIC_VERSION
from sac_forward_task_env import ForwardFirstFlip
from sac_real_start_train_pilot import wrap_env  # includes one-device JAX compat
from sac_residual_real_start_env import ResidualFaceUp, POLICY

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=100000)
    parser.add_argument('--num-envs', type=int, default=32)
    parser.add_argument('--batch-size', type=int, default=128)
    parser.add_argument('--grad-updates', type=int, default=2)
    parser.add_argument('--evals', type=int, default=4)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--name', default='sac_real_start_100k')
    parser.add_argument('--residual', action='store_true')
    parser.add_argument('--residual-scale', type=float, default=.3)
    parser.add_argument('--learning-rate', type=float, default=3e-5)
    parser.add_argument('--stage2-forward', action='store_true',
                        help='Experimental same-run stage-2 reset with directional first-flip reward')
    parser.add_argument('--stage0-forward', action='store_true',
                        help='Pinned actual cube-placement pose after stage 0, directional v2 reward')
    parser.add_argument('--support-aware',action='store_true',help='Experimental measured wood/glass and settled simulated resets')
    parser.add_argument('--reference-residual',help='Reference CEM report for learned position corrections (not a teacher policy)')
    parser.add_argument('--reference-scale',type=float,default=.05)
    parser.add_argument('--direction-sign', type=int, choices=(-1, 1), default=1)
    args = parser.parse_args()
    if args.support_aware and (not args.stage0_forward or args.residual):
        parser.error('--support-aware requires --stage0-forward; residual teacher must be re-audited first')
    if args.reference_residual and (not args.support_aware or args.residual or not 0<args.reference_scale<=.1):
        parser.error('Reference corrections require --support-aware, no teacher residual, and scale in (0,0.1] radians')
    directional=args.stage2_forward or args.stage0_forward
    if args.stage2_forward and args.stage0_forward:
        parser.error('Select one reset stage')
    if args.stage2_forward and args.residual:
        parser.error('Stage-2 teacher has not been evaluated with directional v2')
    if not 0 < args.residual_scale <= 1 or args.learning_rate <= 0:
        parser.error('Residual scale must be in (0,1]; learning rate must be positive')
    teacher_sha=hashlib.sha256(POLICY.read_bytes()).hexdigest() if args.residual else None
    if args.stage0_forward and args.residual:
        audit=json.loads((ROOT/'sac_stage0_teacher_v2_result.json').read_text())
        if (audit.get('metric_version')!=METRIC_VERSION or not audit.get('teacher')
                or audit.get('successes',0)==0 or audit.get('teacher_sha256')!=teacher_sha):
            raise RuntimeError('A matching directional teacher audit with a successful flip is required')
    out = ROOT / args.name
    if out.exists():
        raise RuntimeError(f'Output exists: {out}; choose a new --name')
    out.mkdir()
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    if directional:
        cfg.target_face = 2 if args.direction_sign == 1 else 3
        cfg.chain_goals=False
        cfg.face_onehot_obs=False
        start_class=Stage0StartFaceUp if args.stage0_forward else Stage2StartFaceUp
        task_class=ForwardFirstFlip
        if args.support_aware:
            from sac_supported_start_env import SupportedStart,SupportedForward
            start_class=SupportedStart;task_class=SupportedForward
            smoke=json.loads((ROOT/'sac_supported_smoke_result.json').read_text())
            if not smoke['passed']:raise RuntimeError('Supported MJX smoke check failed')
        train_env = task_class(start_class(
            registry.load('LeapCubeFaceUp', config=cfg)), args.direction_sign)
        eval_env = task_class(start_class(
            registry.load('LeapCubeFaceUp', config=cfg)), args.direction_sign)
        if args.support_aware and smoke['reset_manifest_sha256']!=train_env.reset_manifest_sha256:
            raise RuntimeError('Supported reset changed after smoke check')
        if args.support_aware and smoke.get('dynamics_version')!=train_env.dynamics_version:
            raise RuntimeError('Supported collision model changed after smoke check')
    else:
        train_env = RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg))
        eval_env = RealStartFaceUp(registry.load('LeapCubeFaceUp', config=cfg))
    if args.residual:
        if args.stage0_forward and audit['reset_manifest_sha256']!=train_env.reset_manifest_sha256:
            raise RuntimeError('Teacher audit used a different reset manifest')
        train_env = ResidualFaceUp(train_env,scale=args.residual_scale)
        eval_env = ResidualFaceUp(eval_env,scale=args.residual_scale)
    if args.reference_residual:
        from sac_reference_residual_env import ReferenceResidual
        baseline=json.loads((ROOT/(Path(args.reference_residual).stem+'_mjx_eval.json')).read_text())
        if baseline['successes']==0:raise RuntimeError('Reference has no verified MJX success')
        train_env=ReferenceResidual(train_env,args.reference_residual,args.reference_scale)
        eval_env=ReferenceResidual(eval_env,args.reference_residual,args.reference_scale)
        if baseline['reference_sha256']!=train_env.reference_sha256:raise RuntimeError('Reference baseline hash mismatch')
        if baseline.get('dynamics_version')!=train_env.dynamics_version:raise RuntimeError('Reference collision model mismatch')
    settings = vars(args) | {
        'cube_side_m': .0545, 'cube_mass_kg': .100,
        'gravity_sim_m_s2_candidate': [float(x) for x in train_env.mj_model.opt.gravity],
        'start_offset_candidates': int(train_env.offsets_m.shape[0]),
        'directional_task_experimental': directional,
        'direction_metric_version': METRIC_VERSION if directional else None,
        'start_phase': 'derived_settled_support_candidate' if args.support_aware else 'place_cube_after_stage_0' if args.stage0_forward else 'stage_2' if args.stage2_forward else 'legacy_real_start',
        'reset_manifest_sha256': train_env.reset_manifest_sha256 if args.stage0_forward else None,
        'friction_scale': train_env.friction_scale if directional else 1.0,
        'hardware_authorized': False,
        'teacher_sha256': teacher_sha,
        'dynamics_version': train_env.dynamics_version if args.support_aware else None,
        'reference_sha256': train_env.reference_sha256 if args.reference_residual else None,
        'deterministic_eval': True,
    }
    (out / 'settings.json').write_text(json.dumps(settings, indent=2) + '\n')
    metrics_path = out / 'metrics.jsonl'

    def progress(steps, metrics):
        numbers = {}
        for key, value in metrics.items():
            try:
                numbers[key] = float(value)
            except (TypeError, ValueError):
                pass
        with metrics_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps({'steps': int(steps), 'metrics': numbers}) + '\n')
        print('progress', steps,
              {k: v for k, v in numbers.items() if 'reward' in k or 'success' in k},
              flush=True)

    started = time.monotonic()
    sac.train(
        environment=train_env,
        eval_env=eval_env,
        num_timesteps=args.timesteps,
        episode_length=300,
        num_envs=args.num_envs,
        num_eval_envs=16,
        batch_size=args.batch_size,
        min_replay_size=max(1024, args.batch_size * 8),
        max_replay_size=max(50000, args.timesteps),
        grad_updates_per_step=args.grad_updates,
        num_evals=args.evals,
        learning_rate=args.learning_rate,
        discounting=.99,
        normalize_observations=True,
        deterministic_eval=True,
        wrap_env_fn=wrap_env,
        checkpoint_logdir=str(out / 'checkpoints'),
        progress_fn=progress,
        seed=args.seed,
    )
    report = {'status': 'offline_sac_training_complete',
              'hardware_authorized': False,
              'timesteps': args.timesteps,
              'elapsed_seconds': time.monotonic() - started,
              'checkpoint_dir': str(out / 'checkpoints'),
              'metrics_path': str(metrics_path),
              'caveat': 'Training completion is not physical flip validation.'}
    (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
