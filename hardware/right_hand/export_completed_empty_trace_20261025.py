"""Export measured, separately executed path segments for offline diagnosis."""
import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'
GOALS=ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/quarter_turn_goals_v1.json'
raw=np.asarray(json.loads(GOALS.read_bytes())['raw_goal_ticks'])
paths=[ROOT/'reverse_quarter_path_empty_20261025_082749_355962/result.json',ROOT/'reverse_quarter_resume_empty_20261025_090335_740374/result.json']
records=[];segments=[];ph='stage_6_empty_quarter_reference_path'
for number,path in enumerate(paths):
    r=json.loads(path.read_bytes());rows=[s for s in r['samples'] if s['phase']==ph]
    if number==1:rows += [next(s for s in r['samples'] if s['phase']=='stage_4_empty_start_hold')]
    mapped=[];last_index=0 if number==0 else 472
    for s in rows:
        matches=np.flatnonzero(np.all(raw==s['goal'],axis=1));matches=matches[matches>=last_index]
        if not len(matches):raise ValueError('No monotonic exact source goal match')
        index=int(matches[0]);last_index=index
        if s['goal_readback']!=s['goal']:raise ValueError('Unconfirmed goal in actual segment')
        row=dict(segment=number,index=index,actual=s['actual'],goal=s['goal'],t=s['t'],current_ma=s['current_ma'])
        mapped.append(row);records.append(row)
    segments.append(dict(source=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        minimum_index=mapped[0]['index'],maximum_index=mapped[-1]['index'],records=len(mapped),actual_duration=mapped[-1]['t']-mapped[0]['t']))
prefix_last=next(r for r in reversed(records) if r['segment']==0)
resume_first=next(r for r in records if r['segment']==1)
join_difference=(np.array(resume_first['actual'])-prefix_last['actual']).tolist()
indices=np.unique([r['index'] for r in records]);measured=np.array([next(r['actual'] for r in reversed(records) if r['index']==i) for i in indices])
dense=np.array([np.interp(np.arange(len(raw)),indices,measured[:,j]) for j in range(16)]).T
summary=dict(goal_source=GOALS.relative_to(ROOT).as_posix(),goal_sha256=hashlib.sha256(GOALS.read_bytes()).hexdigest(),segments=segments,
    measured_goal_indices=indices.tolist(),missing_goal_indices=sorted(set(range(len(raw)))-set(indices)),
    actual_splice_difference_ticks=join_difference,maximum_splice_difference_ticks=max(abs(x) for x in join_difference),
    sampled_separately_with_intervening_off=True,continuous_actual_recording=False,
    interpolated_missing_samples_are_not_measurements=True,trajectory_time_axis='Nominal goal index * 0.05s; not wall time.',
    diagnostic_replay_only=True,loaded_hand_response_verified=False,physical_cube_rotation_verified=False,
    raw_records=records)
(OUT/'measured_segments.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
np.savez_compressed(OUT/'goal_indexed_actual_path.npz',goal_ticks=raw,actual_ticks=dense,measured_indices=indices,measured_ticks=measured,
    actual_raw_ticks=np.array([r['actual'] for r in records]))
print(json.dumps({k:v for k,v in summary.items() if k not in ('raw_records','measured_goal_indices','missing_goal_indices')},ensure_ascii=False))
print('MISSING_INDEX_COUNT',len(summary['missing_goal_indices']),'FIRST_LAST',summary['missing_goal_indices'][:10],summary['missing_goal_indices'][-10:])
