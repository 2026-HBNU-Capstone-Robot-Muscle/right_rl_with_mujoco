"""Make an offline request from the user's newly connected OFF readings."""
import hashlib,json
from collections import Counter
import numpy as np
from pitch_handoff_plan_20261028 import PitchHandoffPlan,ROOT,OUT,REQUEST,prefix_rows,loaded_rows,LOADED_PATH,PITCH,pitch_start
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from observed_book_model_policy import healthy_off

def main():
    source=ROOT/'prefix638_off_readonly_20261028_211451_732776/result.json'
    r=json.loads(source.read_bytes())
    assert r['status']=='off_snapshots_saved' and r['maximum_drift_ticks']<=2
    assert r['servo_writes']==0 and r['torque_changes']==0
    for state in r['snapshots']:healthy_off(state)
    OUT.mkdir(parents=True,exist_ok=True)
    request=dict(plan_version=PitchHandoffPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
                 off_state=r['snapshots'][-1],source=source.relative_to(ROOT).as_posix(),
                 source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),hardware_execution_allowed=False)
    REQUEST.write_text(json.dumps(request,indent=2)+'\n',encoding='utf-8')
    print('CURRENT_OFF',json.dumps(dict(position=request['off_state']['position'],
          voltage=request['off_state']['voltage'],temperature=request['off_state']['temperature'])),flush=True)
    p=PitchHandoffPlan(REQUEST);raw=p.source_position();rows=p.plan(raw)
    loaded=loaded_rows()
    np.savez_compressed(LOADED_PATH,start=pitch_start(),goals=np.asarray([q for _,q in loaded]),
                        phases=np.asarray([phase for phase,q in loaded]),sample_period_s=.075)
    count=len(np.load(PITCH)['goals']);values=np.asarray([q for _,q in loaded])
    np.savez_compressed(OUT/'transfer_path.npz',start=values[count-1],goals=values[count:count+600],sample_period_s=.075)
    print(json.dumps(dict(source=source.parent.name,raw_ticks=raw.tolist(),
                         voltage_range_v=[min(request['off_state']['voltage'])/10,max(request['off_state']['voltage'])/10],
                         max_temperature_c=max(request['off_state']['temperature']),
                         queued_goals=len(rows),empty_goals=len(prefix_rows(p,raw)),
                         nominal_motion_seconds=len(rows)*.075,
                         maximum_thumb_travel_ticks=int(np.max(abs(np.array([q for _,q in rows])[:,:4]-raw[:4]))),
                         phases=dict(Counter(phase for phase,q in rows)),hardware_execution_allowed=False)))

if __name__=='__main__':main()
