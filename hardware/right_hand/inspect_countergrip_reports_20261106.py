"""Print report schemas and final trace for local offline research review."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent/'right_hand_rotation_20261105/canonical_body_grip'
for pattern in ('lower_countergrip_station_force2_span12_equilibrium/direct_replay_review.json',):
    for p in sorted(R.glob(pattern)):
        d=json.loads(p.read_bytes());cases=d.get('cases',[])
        print(json.dumps(dict(file=p.relative_to(R).as_posix(),keys=list(d),case0=None if not cases else {k:(list(v[0]) if v and isinstance(v,list) and isinstance(v[0],dict) else type(v).__name__) for k,v in cases[0].items()},last_trace=d.get('trace',[])[-1:])),flush=True)
        if 'direct_replay' in p.name:
            for c in cases:
                v=c.get('dynamic')
                if v:print(json.dumps(dict(seed=c['seed'],completed=v['completed_goals'],total=v['total_goals'],worst=v['worst'],yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],contacts=v['final_contact_N'],reasons=c['reasons'])),flush=True)
        if 'initial_station' in p.as_posix():
            for c in cases:
                v=c.get('dynamic')
                if v:print(json.dumps(dict(offsets=[c['initial_hand_X_offset_mm'],c['initial_hand_Y_offset_mm']],yaw=v['final_yaw_from_placement_deg'],tilt=v['final_up_tilt_deg'],center=v['final_center_world_m'],contacts=v['final_contact_N'],tail=v['settled_tail']['minimum_separating_plane_gap_1ms_mm'],safe=v['safety_passed'])),flush=True)
