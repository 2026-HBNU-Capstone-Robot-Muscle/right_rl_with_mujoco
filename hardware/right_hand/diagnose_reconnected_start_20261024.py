"""WSL-only saved-pose draft diagnosis. Not an executable motion proof.

The draft is bound to one real read-only endpoint and +/-4 ticks. It allows
only the measured thumb-root initial excess (<=0.09rad) to return monotonically
to the unchanged native model range in the first EMPTY segment. No controller
or native model limits are changed, and production admission is untouched.
"""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from hand_motor_mapping import TICK
from grasped_yaw_plan import GraspedYawPlan,yaw_opening,TURN_SAMPLES,AFTER_SAMPLES
from slow_grasp_demo_plan import OBSERVATION_SAMPLES

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'bus_readonly_diagnose_20261024_043442_559037/result.json'
SOURCE_SHA='d714b22879521f91b7c3343e130385391925387900a68e33b5bed4d25672033f'
blob=SOURCE.read_bytes()
if hashlib.sha256(blob).hexdigest()!=SOURCE_SHA:raise ValueError('Read-only endpoint changed')
READINGS=json.loads(blob)
assert READINGS['hardware_response_check_passed'] and READINGS['servo_write_commands_sent']==0
CENTER=np.array(READINGS['stream_samples'][-1]['position_ticks'])
LIMITS=json.loads((ROOT/'right_hand_rotation_20261023/native_motor_limits.json').read_bytes())
LOWER=np.array(LIMITS['lower_ticks']);UPPER=np.array(LIMITS['upper_ticks'])


def admit(raw):
    a=np.asarray(raw)
    if (a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a))
        or np.any(a<0) or np.any(a>4095) or np.any(abs(a-CENTER)>4)):
        raise ValueError('Draft is restricted to the saved OFF endpoint +/-4 ticks')
    excess=np.maximum(0,np.maximum(LOWER-a,a-UPPER))*TICK
    if np.any(np.delete(excess,3)>1e-6) or excess[3]>.09+1e-6:
        raise ValueError('Only measured thumb-root re-entry up to0.09rad is in this diagnostic')
    return a.astype(np.int64)


def off(state):
    a=admit(state['position'])
    for key in ('torque','error','watchdog','voltage','temperature'):
        value=np.asarray(state[key])
        if value.shape!=(16,) or not np.isfinite(value).all() or np.any(value!=np.rint(value)):
            raise ValueError('Invalid saved OFF health state')
    if (any(np.any(state[k]) for k in ('torque','error','watchdog'))
        or np.any(np.array(state['voltage'])<38) or np.any(np.array(state['voltage'])>65)
        or np.any(np.array(state['temperature'])>45)):
        raise ValueError('Saved OFF state is not healthy')
    return a


class DraftPlan(GraspedYawPlan):
    validate_off_state=staticmethod(off)
    MAX_START_RANGE_EXCESS_RAD=.09
    # This extra hash prevents these draft reports matching the production
    # class, even if a future class happened to generate identical waypoints.
    EXTRA_PLANNER_FILES=GraspedYawPlan.EXTRA_PLANNER_FILES+('diagnose_reconnected_start_20261024.py',)

    def validate_start(self,raw):
        a=admit(raw)
        if np.any(abs(a-self.center)>4):raise ValueError('Moved outside exact draft screen neighborhood')
        opening=yaw_opening(a,self._opened)
        loaded=max(20,int(np.ceil(max(abs(self._closed-self._opened))/2)))
        if (len(opening)+loaded+OBSERVATION_SAMPLES+TURN_SAMPLES+AFTER_SAMPLES+20)*.06+2>75:
            raise ValueError('Draft route exceeds unchanged75s budget')
        return a


def main():
    from thumb_live_start_gate import write_request
    folder=ROOT/'right_hand_rotation_20261024/grasped_yaw_reconnected_readonly_fixture'
    folder.mkdir(exist_ok=True)
    row=READINGS['stream_samples'][-1]
    state=dict(position=CENTER,torque=np.array(row['torque']),error=np.array(row['hardware_error']),
        watchdog=np.array(row['watchdog']),voltage=np.rint(np.array(row['voltage_v'])*10).astype(int),
        temperature=np.array(row['temperature_c']))
    generated=write_request(folder,state,DraftPlan.PLAN_VERSION,off)
    r=json.loads(generated.read_bytes());r.update(origin='saved_OFF_draft_diagnostic_not_motion_approval',
        saved_source='bus_readonly_diagnose_20261024_043442_559037/result.json',saved_source_sha256=SOURCE_SHA)
    request=folder/'draft_request.json';request.write_text(json.dumps(r,indent=2),encoding='utf-8')
    output=folder/'draft_diagnostic_only.json'
    assert r['off_state']['position']==CENTER.tolist()
    print('DRAFT goals',len(DraftPlan(request).plan(CENTER)),flush=True)
    import screen_thumb_live_start as checker
    checker.GraspedYawPlan=DraftPlan
    checker.main(request,output)
    result=json.loads(output.read_bytes())
    result.update(diagnostic_only=True,hardware_execution_allowed=False,production_admission_changed=False,
        draft_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(accepted=result['accepted'],diagnostic_only=True,hardware_execution_allowed=False,
        maximum_static_unapproved_contact_mm=max(v['self_escape_max_unapproved_overlap_mm'] for v in result['static_screens']),
        maximum_static_support_increase_mm=max(v['max_support_increase_mm'] for v in result['static_screens']),
        dynamic_tracking=[v['max_tracking_rad'] for v in result['dynamic_sensitivity']]),indent=2))


if __name__=='__main__':main()
