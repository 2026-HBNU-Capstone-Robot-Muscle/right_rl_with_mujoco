"""Read latest empty path result and summarize stationary joints. No devices."""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    d=json.loads((ROOT/'leap_sac_path_empty_result.json').read_text(encoding='utf-8'))
    rows=[r for r in d.get('samples',[]) if r['phase']=='reachable_empty_path']
    out={k:d.get(k) for k in ('timestamp','status','error','path_revision','path_commands_sent','all_torque_off_verified','settings_restored')}
    out['tail']=[dict(t=r['t'],goal8=r['goal'][7],actual8=r['actual'][7],current8=r['current_ma'][7],
                      goal12=r['goal'][11],actual12=r['actual'][11],current12=r['current_ma'][11],
                      goal15=r['goal'][14],actual15=r['actual'][14],current15=r['current_ma'][14],
                      goal16=r['goal'][15],actual16=r['actual'][15],current16=r['current_ma'][15]) for r in rows[-12:]]
    if len(rows)>=6:
        w=rows[-6:];actual=np.array([r['actual'] for r in w]);goal=np.array([r['goal'] for r in w]);current=np.array([r['current_ma'] for r in w])
        out['window_seconds']=w[-1]['t']-w[0]['t']
        out['joint_window']=[dict(id=i+1,actual_span_ticks=int(np.ptp(actual[:,i])),goal_change_ticks=int(goal[-1,i]-goal[0,i]),
                                  lag_ticks=int(goal[-1,i]-actual[-1,i]),current_first=int(current[0,i]),current_last=int(current[-1,i])) for i in range(16)]
    out['motor_commands_sent']=0
    (ROOT/'sac_latest_path_diagnosis_result.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))

if __name__=='__main__':main()
