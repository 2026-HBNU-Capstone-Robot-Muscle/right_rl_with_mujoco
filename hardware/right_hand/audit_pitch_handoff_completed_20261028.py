"""Audit existing command evidence; never opens serial ports or cameras."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np
from pitch_handoff_plan_20261028 import PitchHandoffPlan

ROOT = Path(__file__).resolve().parent
RUN = ROOT / 'pitch_handoff_observe_20261028_221727_181336'
OUT = RUN / 'offline_review'

def main():
    r = json.loads((RUN/'result.json').read_bytes())
    request = RUN / Path(r['request_file'].replace('\\', '/')).name
    plan = PitchHandoffPlan(request)
    rows = plan.plan(np.asarray(r['accepted_enable_goal'], dtype=int))
    expected = np.asarray([q for phase,q in rows])
    goals = [w for w in r['write_events'] if w['address'] == 116]
    actual = np.asarray([w['requested'] for w in goals])
    np.testing.assert_array_equal(actual[2:], expected)
    assert len(actual) == len(expected) + 2
    np.testing.assert_array_equal(actual[-1], r['final_command'])
    assert all(w['comm'] == 0 for w in goals)
    assert r['all_torque_off_verified'] and r['settings_restored']
    assert all(e['status']=='confirmed' and e['clean_reads']>=2 and e['goal_writes']==0 and e['goals_advanced']==0 for e in r.get('read_recovery_confirmation_events', []))
    assert all(e['elapsed_ms'] <= 200 for e in r.get('read_recovery_confirmation_events', []))
    assert all(s['goal']==s['goal_readback'] for s in r['samples'])
    report = dict(status='saved_command_sequence_and_shutdown_verified', source=RUN.name,
        source_result_sha256=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest(),
        request=request.name, planned_targets=len(rows), recorded_position_writes=len(goals),
        initial_preload_and_enable_writes=2, exact_ordered_target_match=True,
        goal_packets_resent_during_read_recovery=0,
        all3372_sample_goal_readbacks_exact=True,
        plan_phases=dict(Counter(p for p,q in rows)),
        read_failures=len(r.get('read_failures', [])),
        recovery_confirmations=r.get('read_recovery_confirmation_events', []),
        all_torque_off_verified=True, settings_restored=True, cube_removal_confirmed=r['cube_removal_confirmed'],
        subsequent_yaw_executed=False, devices_opened=False, robot_commands=0,
        raw_physical_flip_verified_unchanged=r['physical_flip_verified'],
        raw_physical_handoff_verified_unchanged=r['physical_handoff_verified'])
    (OUT/'command_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False))

if __name__ == '__main__':
    main()
