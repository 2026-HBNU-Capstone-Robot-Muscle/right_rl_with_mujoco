"""EMPTY-only recorded path diagnostic; no actor, cube, or camera dependency."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import sequence_flip_checked as checked
from leap_sac_start_empty import target_pose as original_target_pose, plan as original_plan, wait_space
from path_progress_guard import check_progress
from empty_path_retime import half_speed
ROOT=Path(__file__).resolve().parent
PATH_SHA='1ade568d81115b8f6c8dc66f86dfb8845f9a1b145e5034f78bbf4a1295583752'
AUDIT_SHA='54e563f9aacda5b087877916743e1148e80f2e59eab63dfbdb2bb35e915143a4'
HOLD_AUDIT_SHA='7ae8c56f41cd2bca8845ff7a46a4e253d2b9eecb5b56db2479d1e83056b38e0e'
BACKOFF_AUDIT_SHA='003235435cb3b07acd487f382654ee262f5bde6391ab713c87a3c85c8d1340d0'

class DiagnosticBus(checked.Bus):
    def snapshot(self):
        # Identical address/length to the checked bus; no extra bus transaction.
        return self.block(64,83,[('torque',64,1),('error',70,1),
            ('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),
            ('goal_pwm',100,2),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),
            ('goal',116,4),('present_pwm',124,2),('current',126,2),('present_velocity',128,4),
            ('position',132,4),('voltage',144,2),('temperature',146,1)])

def target_pose():
    target=original_target_pose();target[15]=3238
    return target

def plan(initial,target):
    phases=original_plan(initial,target)
    thumb=phases[0][1].copy();thumb[:4]=target[:4]
    return [phases[0],('prepare thumb before other fingers',thumb),phases[1],phases[2]]

def load_path():
    content=(ROOT/'sac_reachable_empty_path_id16capped.json').read_bytes()
    if hashlib.sha256(content).hexdigest()!=PATH_SHA:raise ValueError('Empty path changed or validation pending')
    doc=json.loads(content)
    if doc['format']!='leap_reachable_empty_path_v1' or not doc['empty_only'] or doc['live_policy'] or doc['cube_allowed'] or doc['dt_seconds']!=.05:
        raise ValueError('Not the reviewed empty-only path')
    for name,digest in doc['sources_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:raise ValueError('Simulation validation changed: '+name)
    start=checked.validate_raw(doc['start_ticks']);goals=np.array([checked.validate_raw(g) for g in doc['goals_id_order']])
    if not np.array_equal(start,target_pose()) or goals.shape[0]<2:raise ValueError('Path start/length mismatch')
    allgoals=np.r_[start[None],goals]
    if doc.get('id16_goal_cap_ticks')!=3238 or np.any(allgoals[:,15]>3238):raise ValueError('ID16 bounded goal violated')
    if np.max(np.abs(np.diff(allgoals,axis=0)))>doc['max_step_ticks'] or doc['max_step_ticks']>70:raise ValueError('Path jump exceeds reviewed bound')
    if np.any(allgoals[:,[4,8,12]]<doc['root_floor_ticks']):raise ValueError('Root floor violated')
    return doc,goals

def load_diagnostic_path():
    doc,original=load_path()
    content=(ROOT/'sac_half_speed_empty_audit_result.json').read_bytes()
    if hashlib.sha256(content).hexdigest()!=AUDIT_SHA:raise ValueError('Half-speed audit changed; no motion')
    audit=json.loads(content)
    if not audit['all_passed'] or audit['time_scale']!=2 or audit['control_dt']!=.05:
        raise ValueError('Half-speed empty audit failed')
    for name in ('sac_reachable_empty_path_id16capped.json','empty_path_retime.py'):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=audit['sources_sha256'][name]:
            raise ValueError('Half-speed diagnostic source changed: '+name)
    goals=half_speed(target_pose(),original)
    maximum=int(np.max(np.abs(np.diff(np.vstack([target_pose(),goals]),axis=0))))
    if len(goals)!=audit['command_count'] or maximum!=audit['path_max_step_ticks'] or maximum>35:
        raise ValueError('Half-speed audited trajectory mismatch')
    return doc,goals

def replay(bus,command,report,goals,*,now=time.monotonic,sleep=time.sleep,completion_key='path_completed'):
    report['phase']='reachable_empty_path';report['path_write_attempts']=0;report['path_commands_sent']=0
    checked.sample(bus,command,report,report['phase'])
    began=now();timings=[]
    print('EMPTY PATH NOW: keep cube OUT. One pass, then automatic torque OFF.',flush=True)
    for i,goal in enumerate(goals):
        checked.check_cancel()
        if now()-began>30:raise RuntimeError('Empty path exceeded 30s; no target advance')
        if np.max(np.abs(goal-command))>35:raise RuntimeError('Unexpected path jump; no target advance')
        cycle=now();report['path_write_attempts']+=1
        bus.write(116,4,goal,verify=False);command=goal.copy();report['path_commands_sent']+=1
        # Give the joint time to react, then fresh health/readback before the next goal.
        sleep(.02)
        state,errors=checked.sample(bus,command,report,report['phase'])
        check_progress([r for r in report.get('samples',[]) if r['phase']=='reachable_empty_path'])
        remaining=.05-(now()-cycle)
        if remaining>0:sleep(remaining)
        timings.append(now()-cycle);report['path_cycle_seconds']=timings.copy()
        if i%20==19 or i==len(goals)-1:
            worst=int(np.argmax(errors));details=''
            if 'present_pwm' in state:
                pwm=int(checked.signed_current(np.asarray(state['present_pwm']))[worst])
                current=int(checked.signed_current(np.asarray(state['current']))[worst])
                details=f'; ID{worst+1} current {current}mA PWM {pwm*.113:.1f}%'
            print(f'empty_path: {i+1}/{len(goals)}; max tracking {max(errors):.3f}rad{details}',flush=True)
    checked.sample(bus,command,report,report['phase'])
    report[completion_key]=True
    report.update(path_elapsed_seconds=now()-began,
                  path_cycle_median_ms=float(np.median(timings)*1000),path_cycle_max_ms=float(max(timings)*1000))

def load_hold_diagnostic():
    from id12_hold_spec import select_prefix
    doc,goals=load_diagnostic_path()
    content=(ROOT/'sac_id12_hold_audit_result.json').read_bytes()
    if hashlib.sha256(content).hexdigest()!=HOLD_AUDIT_SHA:raise ValueError('Fixed-hold audit changed')
    audit=json.loads(content)
    if not audit['all_passed'] or audit['id12_hold_goal_ticks']!=3290 or audit['hold_seconds']!=1.0:
        raise ValueError('Fixed-hold audit failed')
    for name in ('id12_hold_spec.py','empty_path_retime.py','sac_reachable_empty_path_id16capped.json'):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=audit['sources_sha256'][name]:
            raise ValueError('Fixed-hold source changed: '+name)
    return doc,select_prefix(goals)

def load_backoff_diagnostic():
    from id12_backoff_spec import targets,END_TICKS,STEPS,DT
    doc,prefix=load_hold_diagnostic()
    content=(ROOT/'sac_id12_backoff_audit_result.json').read_bytes()
    if hashlib.sha256(content).hexdigest()!=BACKOFF_AUDIT_SHA:raise ValueError('Backoff audit changed')
    audit=json.loads(content)
    if not audit['all_passed'] or audit['status']!='offline_id12_backoff_empty':raise ValueError('Backoff audit failed')
    if (audit['return_count'],audit['id12_return_goal_ticks'],audit['control_dt'])!=(STEPS,END_TICKS,DT):
        raise ValueError('Backoff specification mismatch')
    for name in ('id12_backoff_spec.py','id12_stiffness_profile.py','id12_hold_spec.py',
                 'empty_path_retime.py','sac_reachable_empty_path_id16capped.json'):
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=audit['sources_sha256'][name]:
            raise ValueError('Backoff source changed: '+name)
    back=targets(prefix[-1]);allgoals=np.vstack([target_pose(),prefix,back])
    if int(np.max(np.abs(np.diff(allgoals,axis=0))))!=audit['path_max_step_ticks']:
        raise ValueError('Backoff trajectory mismatch')
    return doc,prefix

def run_hold_diagnostic(bus,command,report,goals,*,backoff=False):
    from id12_fixed_hold import observe
    replay(bus,command,report,goals,completion_key='diagnostic_prefix_completed')
    if backoff:
        from id12_backoff import run
        run(bus,goals[-1].copy(),report)
    else:observe(bus,goals[-1].copy(),report)

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true')
    p.add_argument('--id12-hold-check',action='store_true')
    p.add_argument('--id12-stiffness-check',action='store_true')
    p.add_argument('--id12-backoff-check',action='store_true');args=p.parse_args()
    if args.id12_backoff_check:args.id12_stiffness_check=True
    if args.id12_stiffness_check:args.id12_hold_check=True
    if args.execute:
        from hand_motor_mapping import reject_legacy_sac_execution
        reject_legacy_sac_execution()
    doc,goals=load_backoff_diagnostic() if args.id12_backoff_check else load_hold_diagnostic() if args.id12_hold_check else load_diagnostic_path();target=target_pose()
    stem='leap_id12_backoff_empty' if args.id12_backoff_check else 'leap_id12_stiffness_empty' if args.id12_stiffness_check else 'leap_id12_hold_empty' if args.id12_hold_check else 'leap_sac_path_empty'
    print('ID12 RETURN CHECK: empty preparation -> 40 goals -> HOLD 1s -> ID12 RETURN -> HOLD 1s -> OFF.' if args.id12_backoff_check else
          'ID12 HOLD CHECK: empty preparation -> first 40 goals -> FIXED 1s -> OFF. No remaining path.'
          if args.id12_hold_check else 'EMPTY SAC-DERIVED PATH CHECK: preparation -> refined recorded path -> OFF.',flush=True)
    print('ID12 hold diagnostic v1: half-speed prefix only; active PWM/velocity recorded.' if args.id12_hold_check
          else 'Revision 4: HALF-SPEED EMPTY diagnostic; same path, interpolated midpoints; active PWM/velocity recorded.',flush=True)
    print('ID16 path/start <=3238 ticks; thumb first; tracking/current/voltage stops unchanged.',flush=True)
    if args.id12_stiffness_check:
        print('ID12 comparison: P600/I0/D150; Goal Current225mA; measured trip250mA. Others original450mA/PID.',flush=True)
        print('Settings applied only while OFF, restored after OFF. No cube/live SAC.',flush=True)
    else:print(f'No cube, no live SAC. Path {len(goals)*.05:.2f}s nominal; 450mA; original sequence PID.',flush=True)
    if args.id12_backoff_check:
        print('After hold at3290, ID12 alone returns to3130 in32 steps (1.6s nominal), then fixed1s. No forward retry.',flush=True)
    elif args.id12_hold_check:print('ID12 final goal3290 ticks; no new goals or gain changes during hold.',flush=True)
    if not args.execute:print('CHECK ONLY: files validated; no serial/camera opened.');return 0
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='not_started',phase='preflight',
                mode='id12_fixed_hold_empty' if args.id12_hold_check else 'reachable_empty_path',
                empty_only=True,policy_used=False,policy_actions_sent=0,
                path_sha256=PATH_SHA,path_revision=4,time_scale=2,audit_sha256=AUDIT_SHA,
                id16_goal_cap_ticks=3238,preparation_order='thumb_first',loaded_flip_validated=False,
                all_torque_off_verified=False,settings_restored=False,path_completed=False)
    if args.id12_hold_check:report.update(hold_audit_sha256=HOLD_AUDIT_SHA,hold_diagnostic_version=1)
    if args.id12_stiffness_check:report.update(mode='id12_stiffness_empty',stiffness_comparison=True)
    if args.id12_backoff_check:report.update(mode='id12_backoff_empty',backoff_audit_sha256=BACKOFF_AUDIT_SHA,stiffness_comparison=False)
    bus=None;opened=False
    try:
        wait_space()
        import dynamixel_sdk as sdk
        bus=DiagnosticBus(sdk,report)
        if args.id12_stiffness_check:
            from id12_stiffness_profile import make_profile
            bus.checked_control_profile=make_profile()
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set 3000000 baud')
        initial=bus.preflight();poses=plan(initial['position'],target)
        for n in (3,2,1):
            checked.check_cancel();print(f'Starting in {n}s; EMPTY hand. Q/Esc/Ctrl+C stops.',flush=True);time.sleep(1)
        fresh=bus.preflight()
        if np.max(np.abs(fresh['position']-initial['position']))*checked.TICK>.025:raise RuntimeError('Hand moved during countdown')
        checked.run(bus,fresh,poses,report,empty_check=True,
                    prepared_callback=lambda b,c,r:(run_hold_diagnostic(b,c,r,goals,backoff=args.id12_backoff_check) if args.id12_hold_check else replay(b,c,r,goals)))
        report['status']='id12_backoff_complete' if args.id12_backoff_check else 'id12_hold_complete' if args.id12_hold_check else 'empty_path_complete'
        print('ID12 return observation complete. All torque OFF verified; settings restored.' if args.id12_backoff_check else
              'Fixed-hold observation complete. All torque OFF verified; settings restored.' if args.id12_hold_check
              else 'Empty path complete. All torque OFF verified; settings restored.',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted',failed_phase=report.get('phase'))
        print('STOP:',report['error'],flush=True)
    finally:
        if bus is not None and opened:
            try:
                states=bus.block(64,1,[('torque',64,1)])['torque']
                report['all_torque_off_verified']=bool(np.all(states==0));report['final_torque_states']=states.tolist()
            except Exception as exc:
                report['all_torque_off_verified']=False;report['final_check_error']=str(exc)
            if not report['all_torque_off_verified']:print('CUT MOTOR POWER NOW: OFF unverified.',flush=True)
            bus.port.closePort()
        stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        for name in (f'{stem}_result.json',f'{stem}_{stamp}.json'):
            (ROOT/name).write_text(json.dumps(report,ensure_ascii=False,indent=2,default=float),encoding='utf-8')
        if report.get('samples'):
            rows=report['samples']
            print(f'Measured: min voltage {min(min(r["voltage_v"]) for r in rows):.1f}V; '
                  f'peak current {max(abs(v) for r in rows for v in r["current_ma"])}mA; '
                  f'max tracking {max(max(r["tracking_rad"]) for r in rows):.3f}rad.',flush=True)
        if report.get('path_completed'):
            print(f'Path timing: median {report["path_cycle_median_ms"]:.1f}ms; max {report["path_cycle_max_ms"]:.1f}ms.',flush=True)
        print('Saved:',ROOT/f'{stem}_result.json',flush=True)
    expected='id12_backoff_complete' if args.id12_backoff_check else 'id12_hold_complete' if args.id12_hold_check else 'empty_path_complete'
    return 0 if report['status']==expected and report['all_torque_off_verified'] and report['settings_restored'] else 1

if __name__=='__main__':raise SystemExit(main())
