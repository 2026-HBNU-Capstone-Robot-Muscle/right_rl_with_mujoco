"""Batched support-aware evaluation with durable progress (offline only)."""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
import jax
import jax.numpy as jp
import numpy as np
from brax.training.agents.sac import checkpoint
from mujoco_playground import registry
from sac_supported_start_env import SupportedStart,SupportedForward
from sac_residual_real_start_env import JaxTeacher,POLICY
from sac_forward_geometry import METRIC_VERSION
ROOT=Path(__file__).resolve().parent

def save(path,payload):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(payload,indent=2)+'\n');tmp.replace(path)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run');p.add_argument('--teacher',action='store_true')
    p.add_argument('--reference',help='Zero SAC correction baseline around this candidate reference')
    p.add_argument('--diagnostic-resets', help='Offline alternate reset JSON; keeps training reset immutable')
    p.add_argument('--diagnostic-cube-side-mm',type=float,help='Explicit geometry transfer test; never overwrites nominal evaluation')
    p.add_argument('--position-bias-sweep-mm',type=float,help='Observe unchanged physical starts with zero and +/-XYZ camera position biases')
    p.add_argument('--max-steps',type=int,default=180);p.add_argument('--hold-seconds',type=float,default=1.)
    args=p.parse_args()
    if sum([bool(args.run),args.teacher,bool(args.reference)])!=1:p.error('Choose exactly one --run, --teacher, or --reference')
    if args.diagnostic_cube_side_mm is not None and not args.diagnostic_resets:p.error('Changed cube geometry requires screened alternate resets')
    started=time.monotonic();settings={};selected=None
    if args.reference:
        actor=lambda obs,key:(jp.zeros(16),{});output=ROOT/(Path(args.reference).stem+'_mjx_eval.json')
    elif args.teacher:
        teacher=JaxTeacher();actor=lambda obs,key:(teacher(obs),{})
        output=ROOT/'sac_supported_teacher_batch_eval.json'
    else:
        folder=ROOT/args.run;settings=json.loads((folder/'settings.json').read_text())
        if not settings.get('support_aware') or settings['direction_metric_version']!=METRIC_VERSION:raise RuntimeError('Wrong environment/metric checkpoint')
        selected=max((folder/'checkpoints').glob('[0-9]*'),key=lambda p:int(p.name))
        actor=checkpoint.load_policy(str(selected),deterministic=True);output=folder/'supported_batch_eval.json'
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';cfg.chain_goals=False;cfg.face_onehot_obs=False
    env=SupportedForward(SupportedStart(registry.load('LeapCubeFaceUp',config=cfg),diagnostic_cube_side_mm=args.diagnostic_cube_side_mm))
    if settings and settings.get('dynamics_version')!=env.dynamics_version:raise RuntimeError('Checkpoint collision model differs; do not silently reinterpret old results')
    reference=args.reference or settings.get('reference_residual')
    if reference:
        from sac_reference_residual_env import ReferenceResidual
        env=ReferenceResidual(env,reference,scale=settings.get('reference_scale',.05))
        if settings and settings.get('reference_sha256')!=env.reference_sha256:raise RuntimeError('Reference changed since training')
    if settings and settings['reset_manifest_sha256']!=env.reset_manifest_sha256:raise RuntimeError('Reset hash mismatch')
    diagnostic_meta={}
    if args.diagnostic_resets:
        alternate_path=ROOT/args.diagnostic_resets
        alternate=json.loads(alternate_path.read_text())
        if args.diagnostic_cube_side_mm is not None and alternate.get('cube_side_mm')!=args.diagnostic_cube_side_mm:
            raise RuntimeError('Alternate resets were not screened with this cube dimension')
        candidates=alternate['candidates']
        if not candidates or not all(c['passed'] for c in candidates):raise RuntimeError('Alternate reset screening failed')
        base=env
        while not isinstance(base,SupportedStart):base=base.env
        base.initial_q=jp.asarray([c['qpos'] for c in candidates])
        base.initial_ctrl=jp.asarray([c['ctrl'] for c in candidates])
        base.offsets_m=jp.asarray([c['offset_mm'] for c in candidates])/1000
        if base.initial_q.shape!=(len(candidates),23) or base.initial_ctrl.shape!=(len(candidates),16):raise RuntimeError('Alternate reset shape mismatch')
        if not np.isfinite(np.asarray(base.initial_q)).all() or not np.isfinite(np.asarray(base.initial_ctrl)).all():raise RuntimeError('Nonfinite alternate reset')
        diagnostic_meta=dict(diagnostic_resets=args.diagnostic_resets,
                             diagnostic_resets_sha256=hashlib.sha256(alternate_path.read_bytes()).hexdigest(),
                             derived_reset_specs=alternate.get('specs'),
                             warning='Additional simulated starts; no hardware placement tolerance certified')
        output=output.with_name(output.stem+'_'+alternate_path.stem+'.json')
    if args.diagnostic_cube_side_mm is not None:
        diagnostic_meta.update(diagnostic_cube_side_mm=args.diagnostic_cube_side_mm,
                               diagnostic_cube_physics=env.diagnostic_cube_physics,
                               geometry_transfer_test=True)
        output=output.with_name(output.stem+f'_cube{args.diagnostic_cube_side_mm:g}mm.json')
    bias_cases=[[0.,0.,0.]]
    if args.position_bias_sweep_mm is not None:
        if not reference:raise ValueError('Position bias adapter requires the 110-input reference policy')
        from sac_sensor_bias import position_bias_cases
        bias_cases=position_bias_cases(args.position_bias_sweep_mm)
        output=output.with_name(output.stem+f'_positionbias{args.position_bias_sweep_mm:g}mm.json')
        diagnostic_meta.update(camera_position_bias_cases_mm=bias_cases,
                               sensor_bias_scope='Constant cube XYZ observation error; physical state, goal and success metric unchanged. Orientation/occlusion not tested.')
    specs=[(i,s,b) for b in range(len(bias_cases)) for i in range(len(env.offsets_m)) for s in (0,1)];n=len(specs)
    indices=jp.asarray([i for i,s,b in specs]);seeds=jp.asarray([s for i,s,b in specs]);required=3+int(np.ceil(args.hold_seconds/env.dt))
    observation_offsets=None
    if args.position_bias_sweep_mm is not None:
        from sac_sensor_bias import position_bias_offset_mm
        observation_offsets=jp.asarray(np.stack([position_bias_offset_mm(bias_cases[b]) for i,s,b in specs]))
    reset=jax.jit(jax.vmap(env.reset_at));step=jax.vmap(env.step);act=jax.vmap(actor)
    state=reset(jax.vmap(jax.random.PRNGKey)(seeds),indices)
    active=jp.ones(n,dtype=bool);rows=[dict(offset_index=i,seed=s,camera_position_bias_mm=bias_cases[b],trace=[]) for i,s,b in specs]
    for i,row in enumerate(rows):row['trace'].append(dict(step=0,qpos=np.asarray(state.data.qpos[i]).tolist(),ctrl=np.asarray(state.data.ctrl[i]).tolist()))
    print('Reset complete; compiling batched policy step',flush=True)
    @jax.jit
    def block(state,active,first):
        def body(carry,t):
            state,active=carry;keys=jax.vmap(jax.random.PRNGKey)(seeds*1000+t)
            observation=state.obs['state']
            if observation_offsets is not None:observation=observation+observation_offsets
            action=act(observation,keys)[0];new=step(state,action)
            new=jax.tree_util.tree_map(lambda a,b:jp.where(active.reshape((n,)+(1,)*(a.ndim-1)),a,b),new,state)
            physical=(new.done>0)&(new.metrics['forward/success']==0)
            finite=jp.all(jp.isfinite(new.data.qpos),axis=-1)
            hold=new.info['forward_goal_hold'];success=hold>=required;finished=success|physical|~finite
            records=dict(active=active,policy_action=action,qpos=new.data.qpos,ctrl=new.data.ctrl,hold=hold,success=success,physical_done=physical,finite=finite,
                         tip=new.metrics['forward/tip_deg'],axis=new.metrics['forward/off_axis_deg'],error=new.metrics['forward/target_error_deg'],
                         shift=new.metrics['forward/center_shift_m']*1000,support_fault=new.metrics['support/fault'],
                         support_penetration_mm=new.metrics['support/max_penetration_mm'])
            return (new,active&~finished),records
        return jax.lax.scan(body,(state,active),jp.arange(10)+first)
    progress_path=output.with_name(output.stem+'_progress.json')
    meta=dict(status='running',motor_commands_sent=0,metric_version=METRIC_VERSION,reset_manifest_sha256=env.reset_manifest_sha256,
              dynamics_version=env.dynamics_version,
              checkpoint=str(selected) if selected else None,teacher=args.teacher,teacher_sha256=hashlib.sha256(POLICY.read_bytes()).hexdigest() if args.teacher else None,
              control_dt=float(env.dt),required_hold_steps=required,max_steps=args.max_steps,
              reference=reference,reference_sha256=env.reference_sha256 if reference else None,zero_correction_reference=bool(args.reference),**diagnostic_meta)
    save(progress_path,meta|dict(completed_step=0))
    for first in range(1,args.max_steps+1,10):
        (state,active),records=block(state,active,jp.asarray(first));records=jax.device_get(records)
        for k in range(10):
            if first+k>args.max_steps:break
            for i,row in enumerate(rows):
                if not records['active'][k,i]:continue
                record={key:np.asarray(value[k,i]).tolist() for key,value in records.items() if key!='active'};record['step']=first+k
                row['trace'].append(record)
        summaries=[]
        for row in rows:
            trace=row['trace'][1:];last=trace[-1] if trace else {}
            summaries.append(dict(offset_index=row['offset_index'],seed=row['seed'],camera_position_bias_mm=row['camera_position_bias_mm'],steps=last.get('step',0),success=any(r['success'] for r in trace),
                                  acquired=any(r['hold']>=3 for r in trace),longest_hold_steps=max([r['hold'] for r in trace]or[0]),
                                  final_tip_deg=last.get('tip'),final_target_error_deg=last.get('error'),final_shift_mm=last.get('shift'),
                                  support_fault=any(r['support_fault'] for r in trace),finite=all(r['finite'] for r in trace)))
        progress=meta|dict(completed_step=min(first+9,args.max_steps),elapsed_seconds=time.monotonic()-started,results=rows,summaries=summaries)
        save(progress_path,progress)
        print(json.dumps(dict(step=progress['completed_step'],active=int(active.sum()),successes=sum(r['success'] for r in summaries))),flush=True)
        if not bool(active.any()):break
    report=progress|dict(status='offline_supported_batch_evaluation_complete',successes=sum(r['success'] for r in summaries),trials=n,
                         caveat='Derived simulated starts; not hardware success. Width/friction and actuator calibration remain provisional.')
    save(output,report);print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2),flush=True)

if __name__=='__main__':main()
