"""Offline inspection only; does not import motor or camera runners."""
import collections
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
FOLDER = ROOT / 'cube_grasp_trial_20261022_123011_432278'

if __name__ == '__main__':
    if '--status' in sys.argv:
        for filename in ('prepared_cube_review_20261022/current_status.json',
                         'right_hand_start_20261019/objective_and_schedule.json'):
            print(filename)
            doc=json.loads((ROOT/filename).read_bytes())
            for k,v in doc.items():
                print(k,json.dumps(v,ensure_ascii=False)[:1500])
        raise SystemExit(0)
    d = json.loads((FOLDER / 'result.json').read_text(encoding='utf-8'))
    if '--fixtures' in sys.argv:
        from grasp_place_plan import GraspPlacePlan, healthy_off
        import numpy as np
        base=ROOT/'prepared_cube_review_20261022'
        for suffix in ('fixture','prior_fixture','after_stop_fixture','first_stop_fixture'):
            source=base/('grasp_trial_v4_'+suffix)/'request.json'
            dest=base/('grasp_trial_v5_'+suffix)
            dest.mkdir(exist_ok=True)
            request=json.loads(source.read_bytes())
            request['plan_version']=GraspPlacePlan.PLAN_VERSION
            request['offline_replay_source']=str(source.relative_to(ROOT))
            (dest/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        dest=base/'grasp_trial_v5_loaded_stop_fixture';dest.mkdir(exist_ok=True)
        off=d['final_off_snapshots'][-1]
        healthy_off({k:np.asarray(v) for k,v in off.items()})
        request=dict(plan_version=GraspPlacePlan.PLAN_VERSION,
            motor_ids_in_sim_order=d['joint_mapping']['motor_ids_in_sim_order'],
            origin='stored_post_off_snapshot_for_offline_screen_only',off_state=off,
            offline_replay_source=str((FOLDER/'result.json').relative_to(ROOT)),
            continuously_stable_current_hardware_pose_verified=False)
        (dest/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        print('Five offline v5 fixtures written; no hardware accessed.')
        raise SystemExit(0)
    for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored',
              'read_failures','cube_present','final_off_reference_valid'):
        print(k, json.dumps(d.get(k), ensure_ascii=False))
    samples = d.get('samples', [])
    print('PHASE COUNTS', dict(collections.Counter(s['phase'] for s in samples)))
    print('MEASURED', min(min(s['voltage_v']) for s in samples),
          max(abs(x) for s in samples for x in s['current_ma']),
          max(max(s['tracking_rad']) for s in samples))
    writes = d.get('write_events', d.get('writes', []))
    print('WRITES', len(writes))
    for w in [w for w in writes if w['address']==116][-3:]:
        print('LAST GOAL', json.dumps(w, ensure_ascii=False))
    frames = FOLDER / 'camera' / 'frames.json'
    if frames.exists():
        f = json.loads(frames.read_text(encoding='utf-8'))
        rows=f['frames']; loaded=[r for r in rows if r['phase']=='stage_3_loaded_approach']
        print('LOADED CAMERA',len(loaded), [r['image'] for r in loaded[::max(1,len(loaded)//4)]]+[loaded[-1]['image']])
        print('OFF CAMERA', [r['image'] for r in rows[-3:]])
