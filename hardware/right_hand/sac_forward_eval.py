"""Explicit distinct stage-0 resets and trajectory recording for directional v2 SAC."""
import argparse
import hashlib
import json
from pathlib import Path
import jax
import jax.numpy as jp
import numpy as np
from brax.training.agents.sac import checkpoint
from mujoco_playground import registry
from sac_forward_geometry import METRIC_VERSION
from sac_forward_flip_gate import matrix
from sac_ordered_cube_goal import COLORS,NORMALS
from sac_forward_task_env import ForwardFirstFlip
from sac_stage0_start_env import Stage0StartFaceUp
from sac_residual_real_start_env import ResidualFaceUp, POLICY

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--run')
    p.add_argument('--checkpoint',type=int)
    p.add_argument('--seeds',type=int,nargs='+',default=[0,1])
    p.add_argument('--max-steps',type=int,default=180)
    p.add_argument('--hold-seconds',type=float,default=0.,help='Require continuous target hold beyond the initial 3-step acquisition')
    p.add_argument('--offset-indices',type=int,nargs='+')
    p.add_argument('--zero-action',action='store_true')
    p.add_argument('--teacher',action='store_true',help='Audit existing exported teacher under the same corrected goal')
    p.add_argument('--support-aware',action='store_true',help='Use measured support and derived settled resets for baselines')
    args=p.parse_args()
    if args.hold_seconds<0 or args.max_steps<1: p.error('Invalid hold or step limit')
    actor=None; settings={}; selected=None
    if args.zero_action and args.teacher: p.error('Select only one baseline controller')
    if args.teacher:
        from sac_residual_real_start_env import JaxTeacher
        teacher=JaxTeacher()
        actor=jax.jit(lambda obs,key: (teacher(obs),{}))
    elif not args.zero_action:
        if not args.run: p.error('--run required for policy evaluation')
        folder=ROOT/args.run
        settings=json.loads((folder/'settings.json').read_text())
        if settings.get('direction_metric_version')!=METRIC_VERSION or not settings.get('stage0_forward'):
            raise RuntimeError('Requires a stage-0 directional v2 checkpoint; legacy results cannot be silently reinterpreted')
        paths=sorted((folder/'checkpoints').glob('[0-9]*'),key=lambda x:int(x.name))
        selected=(next(x for x in paths if int(x.name)==args.checkpoint)
                  if args.checkpoint is not None else paths[-1])
        actor=jax.jit(checkpoint.load_policy(str(selected),deterministic=True))
    sign=settings.get('direction_sign',1)
    cfg=registry.get_default_config('LeapCubeFaceUp'); cfg.impl='jax'
    cfg.chain_goals=False; cfg.face_onehot_obs=False
    supported=bool(settings.get('support_aware',False) or args.support_aware)
    if settings and args.support_aware and not settings.get('support_aware',False):
        raise RuntimeError('Cannot silently evaluate an old checkpoint as support-aware')
    start_class=Stage0StartFaceUp;task_class=ForwardFirstFlip
    if supported:
        from sac_supported_start_env import SupportedStart,SupportedForward
        start_class=SupportedStart;task_class=SupportedForward
    start=start_class(registry.load('LeapCubeFaceUp',config=cfg))
    if settings and start.reset_manifest_sha256!=settings['reset_manifest_sha256']:
        raise RuntimeError('Reset manifest changed since training')
    env=task_class(start,sign)
    if supported and settings and settings.get('dynamics_version')!=env.dynamics_version:
        raise RuntimeError('Checkpoint collision model mismatch')
    if settings.get('reference_residual'):
        raise RuntimeError('Use sac_supported_batch_eval.py for reference-residual checkpoints')
    teacher_sha=hashlib.sha256(POLICY.read_bytes()).hexdigest() if args.teacher or settings.get('residual') else None
    if settings.get('residual'):
        if settings.get('teacher_sha256')!=teacher_sha:
            raise RuntimeError('Residual teacher changed since training')
        env=ResidualFaceUp(env,scale=settings['residual_scale'])
    reset=jax.jit(env.reset_at); step_fn=jax.jit(env.step)
    hold_steps=3+int(np.ceil(args.hold_seconds/env.dt))
    indices=args.offset_indices if args.offset_indices is not None else range(len(start.offsets_m))
    if any(i<0 or i>=len(start.offsets_m) for i in indices): p.error('Invalid offset index')
    rows=[]
    for index in indices:
        for seed in args.seeds:
            state=reset(jax.random.PRNGKey(seed),jp.asarray(index))
            R0=matrix(np.asarray(state.data.qpos[-4:]))
            up,front=R0@NORMALS[5],R0@NORMALS[3]
            trace=[dict(step=0,qpos=np.asarray(state.data.qpos).tolist(),
                        ctrl=np.asarray(state.data.ctrl).tolist())]
            first_success=None; acquired=None; longest_hold=0
            for step in range(1,args.max_steps+1):
                action=(jp.zeros(env.action_size) if actor is None else
                        actor(state.obs['state'],jax.random.PRNGKey(seed*1000+step))[0])
                state=step_fn(state,action)
                trace.append(dict(step=step,qpos=np.asarray(state.data.qpos).tolist(),
                                  ctrl=np.asarray(state.data.ctrl).tolist(),
                                  tip_deg=float(state.metrics['forward/tip_deg']),
                                  off_axis_deg=float(state.metrics['forward/off_axis_deg']),
                                  target_error_deg=float(state.metrics['forward/target_error_deg']),
                                  center_shift_mm=1000*float(state.metrics['forward/center_shift_m'])))
                hold=int(state.info['forward_goal_hold']); longest_hold=max(longest_hold,hold)
                if hold>=3 and acquired is None: acquired=step
                if hold>=hold_steps: first_success=step
                physical_done=bool(state.done) and not bool(state.metrics['forward/success'])
                if first_success is not None or physical_done: break
            R=matrix(np.asarray(state.data.qpos[-4:]))
            top=int(np.argmax((R@NORMALS.T).T@up)); facing=int(np.argmax((R@NORMALS.T).T@front))
            row=dict(offset_index=index,offset_mm=(np.asarray(start.offsets_m[index])*1000).tolist(),
                     seed=seed,steps=step,success_step=first_success,
                     first_acquired_step=acquired,longest_target_hold_seconds=longest_hold*env.dt,
                     final_top=COLORS[top],final_front=COLORS[facing],
                     final_tip_deg=trace[-1]['tip_deg'],final_off_axis_deg=trace[-1]['off_axis_deg'],
                     final_target_error_deg=trace[-1]['target_error_deg'],
                     final_center_shift_mm=trace[-1]['center_shift_mm'],
                     dropped=bool(state.data.qpos[-5]<-.05),trace=trace)
            rows.append(row)
            print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    report=dict(status='offline_stage0_forward_v2_eval',metric_version=METRIC_VERSION,
                motor_commands_sent=0,checkpoint=str(selected) if selected else None,
                reset_manifest_sha256=start.reset_manifest_sha256,
                trials=len(rows),distinct_offsets=len(indices),control_dt=float(env.dt),
                hold_seconds=args.hold_seconds,required_hold_steps=hold_steps,
                successes=sum(r['success_step'] is not None for r in rows),
                zero_action=args.zero_action,teacher=args.teacher,teacher_sha256=teacher_sha,
                residual=bool(settings.get('residual')),results=rows,
                support_aware=supported,
                dynamics_version=env.dynamics_version if supported else None,
                caveat=('Measured wood/glass with assumed extents and derived simulated reset; camera occlusion and actuator calibration remain unresolved. Simulation success is not hardware success.' if supported else 'Approximate real placement model; no calibrated external wood or camera occlusion. Simulation success is not hardware success.'))
    output=(ROOT/'sac_stage0_teacher_v2_result.json' if args.teacher else
            ROOT/'sac_stage0_zero_action_v2_result.json' if args.zero_action else
            ROOT/args.run/f'directional_eval_v2_{int(selected.name)}.json')
    if supported:output=output.with_name(output.stem+'_support'+output.suffix)
    if args.hold_seconds or args.offset_indices is not None:
        suffix=f'_hold{args.hold_seconds:g}'
        if args.offset_indices is not None: suffix+='_offsets'+','.join(map(str,args.offset_indices))
        output=output.with_name(output.stem+suffix+output.suffix)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__=='__main__': main()
