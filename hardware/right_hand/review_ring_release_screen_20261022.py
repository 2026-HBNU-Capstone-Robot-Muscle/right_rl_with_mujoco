"""Read saved failed screens only; no hardware access."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
FOLDERS=('cube_grasp_trial_20261022_125440_000527','cube_grasp_trial_20261022_125613_698937')

if __name__=='__main__':
    if '--fixtures' in sys.argv:
        from grasp_place_plan import GraspPlacePlan
        base=ROOT/'prepared_cube_review_20261022'
        for suffix in ('fixture','prior_fixture','after_stop_fixture','first_stop_fixture','loaded_stop_fixture'):
            source=base/('grasp_trial_v5_'+suffix)/'request.json'
            folder=base/('grasp_trial_v6_'+suffix);folder.mkdir(exist_ok=True)
            request=json.loads(source.read_bytes());request['plan_version']=GraspPlacePlan.PLAN_VERSION
            request['offline_replay_source']=str(source.relative_to(ROOT))
            (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        for name in FOLDERS:
            source=ROOT/name/'fresh_off_request.json'
            folder=base/('grasp_trial_v6_'+name.split('_')[-2]+'_fixture');folder.mkdir(exist_ok=True)
            request=json.loads(source.read_bytes());request['plan_version']=GraspPlacePlan.PLAN_VERSION
            request['offline_replay_source']=str(source.relative_to(ROOT))
            (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        print('Seven v6 offline requests written; original v5 results preserved; no hardware accessed.')
        raise SystemExit(0)
    for name in FOLDERS:
        folder=ROOT/name;r=json.loads((folder/'result.json').read_bytes())
        proof=json.loads((folder/'current_start_screen.json').read_bytes())
        static=proof['static_screens']
        print(name)
        for k in ('status','error','all_torque_off_verified','settings_restored'):
            print(k,r.get(k))
        print('writes',len(r.get('write_events',[])))
        print('raw',proof['start_ticks'])
        print('static max', {k:max((s.get(k) or 0) for s in static) for k in
            ('max_self_overlap_mm','contact_clear_pip_travel_ticks','self_escape_complete_at',
             'self_escape_max_unapproved_overlap_mm','self_escape_max_step_increase_mm',
             'unapproved_native_range_violation_rad','max_support_increase_mm')})
        print('nominal',static[0])
        print('worst',max(static,key=lambda s:s.get('contact_clear_pip_travel_ticks') or -1))
        print('failed count',sum(s['contact_clear_pip_travel_ticks'] is None or s['contact_clear_pip_travel_ticks']>64 for s in static))
