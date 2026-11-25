"""Verify the saved physical command sequence without changing raw results."""
import hashlib,json,sys
from collections import Counter
from pathlib import Path
import numpy as np
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan
from loaded_repeat_plan_20261026 import LoadedRepeatPlan
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'loaded_thumb_rotation_20261026_135418_442474'
if len(sys.argv)>1:SOURCE=(ROOT/sys.argv[1]).resolve()
if not SOURCE.is_relative_to(ROOT):raise ValueError('Saved source must be inside outputs')
def main():
    r=json.loads((SOURCE/'result.json').read_bytes())
    plan_type=LoadedRepeatPlan if r['plan_version']==LoadedRepeatPlan.PLAN_VERSION else LoadedThumbRotationPlan
    p=plan_type(SOURCE/'fresh_off_request.json')
    rows=p.plan(np.asarray(r['accepted_enable_goal']));writes=[v for v in r['write_events'] if v['address']==116]
    assert r['status']==p.STATUS and r['all_torque_off_verified'] and r['settings_restored']
    assert r['cube_placement_confirmed'] and r['cube_removal_confirmed'] and not r['cube_present']
    assert len(writes)==len(rows)+2 and all(v['comm']==0 for v in r['write_events'])
    np.testing.assert_array_equal([v['requested'] for v in writes[2:]],[q for phase,q in rows])
    np.testing.assert_array_equal([s['goal_readback'] for s in r['samples']],[s['goal'] for s in r['samples']])
    np.testing.assert_array_equal(r['final_command'],rows[-1][1])
    assert r['camera_recording']['image_signal_blank_frames']==0
    assert all(v['status']=='recovered' and v['healthy_confirmation_count']==2 and v['elapsed_ms']<=150 and v['goals_advanced']==0 for v in r.get('voltage_events',[]))
    out=dict(status='exact_loaded_command_sequence_complete',source=SOURCE.name,source_sha256=hashlib.sha256((SOURCE/'result.json').read_bytes()).hexdigest(),
        plan_version=p.PLAN_VERSION,planned_goals=len(rows),actual_goal_writes=len(writes),all_goals_exactly_once=True,
        all_goal_readbacks_match=True,cube_placement_and_removal_confirmed=True,
        all_torque_off_verified=True,settings_restored=True,
        minimum_voltage_v=min(min(s['voltage_v']) for s in r['samples']),
        maximum_current_ma=max(max(abs(v) for v in s['current_ma']) for s in r['samples']),
        maximum_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
        recovered_voltage_events=r.get('voltage_events',[]),camera=r['camera_recording'],
        final_off_reference_valid=r['final_off_reference_valid'],
        physical_angle_requires_visual_review=True,raw_result_unchanged=True,hardware_commands_sent=0)
    (SOURCE/'command_completion_review.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in out.items() if k not in ('recovered_voltage_events','camera')}),flush=True)
if __name__=='__main__':main()
