"""Compare actual measured joint states at identical16-joint commands."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;E=ROOT/'yaw_load_empty_20261029_055917_615720';L=ROOT/'four_reply_yaw_20261029_052443_385204'
empty=json.loads((E/'result.json').read_bytes());loaded=json.loads((L/'result.json').read_bytes())
def brief(s):return dict(t=s['t'],goal=s['goal'][11],actual=s['actual'][11],error_ticks=s['goal'][11]-s['actual'][11],tracking_rad=s['tracking_rad'][11],current_ma=s['current_ma'][11])
def phases(r):
    out={}
    for phase in dict.fromkeys(s['phase'] for s in r['samples']):
        samples=[s for s in r['samples'] if s['phase']==phase];flat=np.asarray([s['tracking_rad'] for s in samples]);i,j=np.unravel_index(np.argmax(flat),flat.shape)
        out[phase]=dict(samples=len(samples),ID12_max_current=max(abs(s['current_ma'][11]) for s in samples),
            ID12_max_tracking=max(s['tracking_rad'][11] for s in samples),maximum_tracking_motor_id=int(j+1),maximum_tracking_rad=float(flat[i,j]),
            maximum_tracking_sample_goal=samples[i]['goal'][j],maximum_tracking_sample_actual=samples[i]['actual'][j])
    return out
ey=[s for s in empty['samples'] if s['phase']=='stage_7_horizontal_yaw'];ly=[s for s in loaded['samples'] if s['phase']=='stage_7_horizontal_yaw']
lookup={tuple(s['goal']):s for s in ey};pairs=[]
for s in ly:
    a=lookup.get(tuple(s['goal']))
    if a:pairs.append(dict(loaded=brief(s),empty=brief(a),all16_goal_values_match=True))
same_stop=next((p for p in reversed(pairs) if p['loaded']['goal']==ly[-1]['goal'][11]),None)
hold=[s for s in empty['samples'] if s['phase']=='stage_2_fixed_hold']
report=dict(status='measured_identical_command_cube_comparison',empty_source=E.name,loaded_source=L.name,
    empty_sha256=hashlib.sha256((E/'result.json').read_bytes()).hexdigest(),loaded_sha256=hashlib.sha256((L/'result.json').read_bytes()).hexdigest(),
    empty_completed=empty['status']=='empty_yaw_load_comparison_complete',empty_yaw_samples=len(ey),loaded_yaw_samples=len(ly),
    matched_commands=len(pairs),at_loaded_stop_same_command=same_stop,empty_phases=phases(empty),loaded_phases=phases(loaded),
    fixed_end_hold=dict(first=brief(hold[0]),last=brief(hold[-1])) if hold else None,
    empty_OFF_verified=empty['all_torque_off_verified'],empty_settings_restored=empty['settings_restored'],
    comparisons=pairs[::30]+pairs[-1:],grip_force_still_not_measured=True,assistant_motor_writes=0)
O=E/'offline_review';O.mkdir(exist_ok=True);(O/'loaded_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('comparisons',)},ensure_ascii=False))
print('COMPARISONS',json.dumps(report['comparisons']))
