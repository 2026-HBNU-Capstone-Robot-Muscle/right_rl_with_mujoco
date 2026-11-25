import json
from pathlib import Path
R=Path(__file__).resolve().parent
def main():
    p=max(R.glob('resume_off_readonly_20261105_*/result.json'),key=lambda p:p.stat().st_mtime);r=json.loads(p.read_bytes())
    print('READONLY',json.dumps(dict(source=p.relative_to(R).as_posix(),status=r['status'],drift=r.get('maximum_drift_ticks'),torque_off=r.get('all_torque_off_read'),health_errors=r.get('pose_health_validation_errors'),position=r.get('snapshots',[{}])[-1].get('position'))))
    root=R/'right_hand_rotation_20261028/two_axis_connection/operator_pitch_handoff_v3_side_clearance'
    for name in ('loaded_geometry/review.json','loaded_hand_dynamics/review.json','transfer_retention/review.json'):
        p=root/name;d=json.loads(p.read_bytes());cases=d.get('cases',[])
        print(name,json.dumps(dict(keys=[k for k in d if k!='cases'],cases=len(cases),all_passed=d.get('all_passed'),all_retention_passed=d.get('all_retention_cases_passed'),max_raw_self=max((v.get('raw_max_self_mm',0) for v in cases),default=0))))
    from pitch_handoff_plan_20261028 import loaded_rows,PITCH_PHASE,PITCH_HOLD,TRANSFER_PHASE,pitch_start
    rows=loaded_rows();phases={p:sum(x==p for x,q in rows) for p,q in rows};print('PITCH',json.dumps(dict(phases=phases,start=pitch_start().tolist(),last_pitch=next(q.tolist() for p,q in reversed(rows) if p==PITCH_HOLD))))
if __name__=='__main__':main()
