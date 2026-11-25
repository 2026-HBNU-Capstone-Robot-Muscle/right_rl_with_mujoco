"""Offline analysis of fixed-goal trials, no port/camera access."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def summarize(name):
    d=json.loads((ROOT/name).read_text())
    rows=[r for r in d['samples'] if r['phase']=='id12_fixed_goal_hold']
    if not rows:raise ValueError('No fixed-goal samples')
    result=dict(file=name,timestamp=d['timestamp'],status=d['status'],
        torque_off=d['all_torque_off_verified'],settings_restored=d['settings_restored'],
        samples=len(rows),span_s=rows[-1]['t']-rows[0]['t'],
        goal_ticks=sorted(set(r['goal'][11] for r in rows)),
        actual_min=min(r['actual'][11] for r in rows),actual_max=max(r['actual'][11] for r in rows),
        current_median_ma=float(np.median([r['current_ma'][11] for r in rows])),
        current_max_ma=max(abs(r['current_ma'][11]) for r in rows),
        pwm_median_percent=float(np.median([r['present_pwm_raw'][11] for r in rows]))*.113,
        min_voltage_v=min(r['voltage_v'][11] for r in rows),
        error_end_rad=d['fixed_hold']['tracking_end_median_rad'][11])
    return result

def main():
    result=dict(baseline=summarize('leap_id12_hold_empty_result.json'),
        user_observation='No visible external snag/contact reported',
        conclusion='Fixed-goal residual remains; simple transient lag is insufficient. Static load/friction/control stiffness or internal constraints remain possible; cause not proven.',
        motor_commands_sent=0)
    candidate=ROOT/'leap_id12_stiffness_empty_result.json'
    if candidate.exists():result['p600_trial']=summarize(candidate.name)
    (ROOT/'id12_hold_analysis_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
