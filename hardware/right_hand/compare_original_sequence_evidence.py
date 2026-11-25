"""Summarize preserved command/encoder records, without importing hardware code."""
import hashlib
import json
from pathlib import Path
from collections import Counter
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_review'


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    poses=json.loads((ROOT/'sequence_flip_poses.json').read_bytes())['poses']
    records=[]
    for name in ('leap_sequence_flip_20261011_051350_713486.json',
                 'leap_sequence_flip_20261011_051518_433595.json'):
        p=ROOT/name;blob=p.read_bytes();data=json.loads(blob)
        samples=data.get('samples',[]);stages=[]
        for stage in range(7):
            rows=[s for s in samples if s.get('phase')==f'stage_{stage}' and 'actual' in s]
            if not rows:continue
            last=rows[-1];actual=np.asarray(last['actual'],float)
            command=np.array(poses[stage]['raw_ticks'])
            stages.append(dict(stage=stage,samples=len(rows),last_measured_ticks=actual.tolist(),
                original_recorded_ticks=command.tolist(),
                last_actual_minus_recorded_ticks=(actual-command).tolist(),
                maximum_absolute_last_error_ticks=float(max(abs(actual-command)))))
        records.append(dict(file=name,sha256=hashlib.sha256(blob).hexdigest(),
            status=data.get('status'),physical_flip_verified=data.get('physical_flip_verified'),
            policy_used=data.get('policy_used'),sample_count=len(samples),
            sample_keys=list(samples[0]) if samples else [],phase_counts=dict(Counter(s.get('phase') for s in samples)),
            stages=stages))
    result=dict(records=records,
        user_observation_from_conversation=dict(green_face_stayed_up=True,
            front_changed_from_orange_to_yellow_with_leftward_horizontal_rotation=True,
            person_held_base_only=True,vertical_flip_not_confirmed=True,
            ninety_degree_angle_not_independently_measured=True),
        records_unchanged=True,hardware_commands_sent=0,
        interpretation='Historical horizontal motion is a useful baseline, not verified arbitrary-direction 90 degree retention.')
    registration_path=OUT/'same_session_marker_registration.json'
    if registration_path.exists():
        reg=json.loads(registration_path.read_bytes())
        estimates=[]
        for folder in reg['folders']:
            by_image={r['image']:r for r in folder['registered']}
            if not all(n in by_image for n in ('stage_2_end.png','stage_4_end.png')):continue
            a,b=by_image['stage_2_end.png'],by_image['stage_4_end.png']
            A,B=np.array(a['rotation_world']),np.array(b['rotation_world'])
            greenA,greenB=A@np.array([0.,-1.,0.]),B@np.array([0.,-1.,0.])
            orangeA,orangeB=A@np.array([-1.,0.,0.]),B@np.array([-1.,0.,0.])
            estimate=dict(folder=folder['folder'],stages=[2,4],
                total_angle_estimate_deg=float(np.degrees(np.arccos(np.clip((np.trace(B@A.T)-1)/2,-1,1)))),
                green_normal_change_estimate_deg=float(np.degrees(np.arccos(np.clip(greenA@greenB,-1,1)))),
                orange_turn_about_initial_green_estimate_deg=float(np.degrees(np.arctan2(
                    np.cross(orangeA,orangeB)@greenA,orangeA@orangeB))),
                start_cube_marker_ids=a['cube_marker_ids'],end_cube_marker_ids=b['cube_marker_ids'],
                single_marker_start_ambiguity=len(a['cube_marker_ids'])==1,
                measured_precision_verified=False,physical_ninety_degree_success_certified=False)
            estimates.append(estimate)
            print('HISTORICAL_HORIZONTAL_ESTIMATE',estimate)
        result['same_session_rotation_estimates']=estimates
    (OUT/'historical_execution_evidence.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    for row in records:
        print(row['file'],row['status'],'samples',row['sample_count'],'keys',row['sample_keys'])
        print('phase_counts',row['phase_counts'])
        print('end_errors_ticks',[(s['stage'],s['maximum_absolute_last_error_ticks']) for s in row['stages']])


if __name__=='__main__':main()
