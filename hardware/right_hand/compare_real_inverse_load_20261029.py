"""Compare Cube and EMPTY readings at identical recorded joint targets.

Current differences are indicators, not calibrated contact force measurements.
"""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
def main():
    pairs=[('gapped_inverse_trial_20261029_184252_272166','reverse_yaw_empty_20261029_164720_101745'),
           ('supported_inverse_trial_20261029_231345_341471','supported_inverse_empty_20261029_225256_285948')]
    reports=[]
    for loaded,empty in pairs:
        cube=json.loads((ROOT/loaded/'result.json').read_bytes());free=json.loads((ROOT/empty/'result.json').read_bytes())
        phases=[]
        for phase in ('stage_6_grasp_hold','stage_8_horizontal_hold'):
            ll=[s for s in cube['samples'] if s['phase']==phase][-20:];target=np.array(ll[-1]['goal'])
            matches=[s for s in free['samples'] if max(abs(np.asarray(s['goal'])-target))<=1]
            bestphase=max(set(s['phase'] for s in matches),key=lambda p:sum(s['phase']==p for s in matches)) if matches else None
            ee=[s for s in matches if s['phase']==bestphase][-20:]
            row=dict(phase=phase,empty_match_phase=bestphase,empty_samples=len(ee),loaded_samples=len(ll))
            if len(ee)>=5:
                currents=np.median([s['current_ma'] for s in ll],axis=0);base=np.median([s['current_ma'] for s in ee],axis=0)
                position=np.median([s['actual'] for s in ll],axis=0);basepos=np.median([s['actual'] for s in ee],axis=0)
                row.update(loaded_current_mA=currents.tolist(),empty_current_mA=base.tolist(),loaded_minus_empty_mA=(currents-base).tolist(),
                    loaded_minus_empty_actual_ticks=(position-basepos).tolist(),target=target.tolist())
            phases.append(row)
        report=dict(loaded=loaded,empty=empty,phases=phases,contact_force_not_calibrated=True,devices_opened=False)
        reports.append(report);print(json.dumps(report),flush=True)
    out=ROOT/'right_hand_rotation_20261029/real_inverse_load_comparison';out.mkdir(exist_ok=True)
    (out/'review.json').write_text(json.dumps(dict(cases=reports),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
