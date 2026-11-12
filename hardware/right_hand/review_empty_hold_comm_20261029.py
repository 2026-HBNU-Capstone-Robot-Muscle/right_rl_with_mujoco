"""Bounded offline review of the user-run45s communication observation."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'empty_hold_comm_20261029_042239_202853/result.json'
def review():
    r=json.loads(SOURCE.read_bytes());samples=r['samples'];hold=[s for s in samples if s['phase']=='empty_fixed_hold_probe']
    assert r['status']=='empty_hold_comm_observation_complete' and r['empty_hold_complete'] and not r['cube_present']
    assert r['all_torque_off_verified'] and r['settings_restored']
    h=r['empty_fixed_hold_probe'];assert h['elapsed_seconds']>=45 and h['period_s']==.075 and h['goal_writes']==0 and h['checks']>=500
    assert all(s['goal']==hold[0]['goal'] and s['goal_readback']==hold[0]['goal'] for s in hold)
    confirmations=r['read_recovery_confirmation_events']
    assert all(e['status']=='confirmed' and e['clean_reads']==2 and e['elapsed_ms']<=200 and e['goal_writes']==0 for e in confirmations)
    after=r['final_off_snapshots'][-1]
    assert not any(after['torque']) and not any(after['error']) and after['p_gain']==[2100]*16 and after['d_gain']==[1900]*16
    result=dict(status='completed45s_empty_observation_reviewed',source=SOURCE.relative_to(ROOT).as_posix(),
        sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),hold_checks=h['checks'],hold_samples_including_confirmations=len(hold),
        fixed_hold_seconds=h['elapsed_seconds'],sample_period_s=h['period_s'],goals_constant_through_hold=True,
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
        peak_current_ma=max(max(abs(np.asarray(s['current_ma']))) for s in samples).item(),
        read_failures=len(r.get('read_failures',[])),recovery_episodes=len(confirmations),
        longest_recovery_ms=max(e['elapsed_ms'] for e in confirmations),
        recoveries_by_phase=dict(collections.Counter(e['phase'] for e in confirmations)),
        recovery_failures=[dict(phase=e.get('phase'),label=e.get('label'),error=e.get('error'),detail=e.get('detail')) for e in r.get('read_failures',[])],
        camera=r['camera_recording'],torque_OFF_verified=True,original_settings_restored=True,
        OFF_endpoint_was_still_settling=not r['final_off_reference_valid'],root_cause_identified=False,
        broader_motion_or_cube_contact_stability_verified=False,assistant_motor_writes=0)
    return result
if __name__=='__main__':
    r=review();folder=SOURCE.parent/'offline_review';folder.mkdir(exist_ok=True)
    (folder/'review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k not in ('recovery_failures','camera')},ensure_ascii=False))
