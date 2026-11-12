"""Offline full-screen regressions at two actual starts and labeled sag variations."""
import copy,contextlib,io,json
from pathlib import Path
import screen_reverse_place_capture_20261025 as s
from reverse_place_capture_plan_20261025 import ROOT,ReversePlacePlan

OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_recovery_regressions_v8'

def main():
    OUT.mkdir(exist_ok=True)
    actual=json.loads((ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v8/request.json').read_bytes())
    old=json.loads((ROOT/'reverse_start_readonly_20261025_044808_834972/result.json').read_bytes())
    cases=[('earlier_actual_off',old['off_state'],{}),
           ('latest_roots_minus_16',actual['off_state'],{7:-16,11:-16,15:-16}),
           ('latest_roots_plus_16',actual['off_state'],{7:16,11:16,15:16})]
    results=[]
    for name,state,changes in cases:
        request=copy.deepcopy(actual);request['off_state']=copy.deepcopy(state)
        for index,delta in changes.items():request['off_state']['position'][index]+=delta
        request.update(synthetic_sensitivity_only=bool(changes),synthetic_changes_by_raw_array_index=changes,
            recorded_actual_source=('reverse_start_readonly_20261025_044808_834972/result.json' if not changes else actual['actual_readonly_source']))
        folder=OUT/name;folder.mkdir(exist_ok=True);q=folder/'request.json';q.write_text(json.dumps(request,indent=2),encoding='utf-8')
        plan=ReversePlacePlan(q)
        plan.plan(plan.source_position())
        with (folder/'screen.log').open('w',encoding='utf-8') as log,contextlib.redirect_stdout(log):
            code=s.main(q,folder/'screen.json')
        r=json.loads((folder/'screen.json').read_bytes())
        row=dict(name=name,synthetic_sensitivity_only=bool(changes),exit_code=code,
            accepted=r['accepted_for_supervised_empty_preparation'],static_cases=len(r['static_screens']),dynamic_cases=len(r['dynamic_sensitivity']),
            worst_support_separation_seconds=max(d['support_escape_complete_at'] if d['support_escape_complete_at'] is not None else 1e9 for d in r['dynamic_sensitivity']))
        results.append(row);print(json.dumps(row),flush=True)
    (OUT/'summary.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    return 0 if all(r['accepted'] for r in results) else 2

if __name__=='__main__':raise SystemExit(main())
