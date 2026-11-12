"""Portable numerical/transaction checks; no robot, camera or MuJoCo import."""
from pathlib import Path
import json
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from shadow_runtime import ShadowController,SIM_IDS,TICK

ROOT=HERE.parents[1]


def main():
    bundle=HERE/'shadow_bundle_nominal_only';runtime=ShadowController(bundle)
    z=dict(np.load(ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/initial_state.npz',allow_pickle=False))
    sample={'frame_id':'leap_sim_world','palm_point':'policy_palm_position','pose_time':0.,'sample_time':0.,
            'sequence':0,'measured_ticks':runtime.to_motor(z['qpos'][:16]),'commanded_ticks':runtime.start.copy(),
            'palm_position':np.array([.11,0.,.03]),'cube_position':z['qpos'][-7:-4],
            'cube_quaternion':z['qpos'][-4:]}
    checks=[]
    def rejected(fn,name):
        try:fn()
        except ValueError:checks.append(name);return
        raise AssertionError('Not rejected: '+name)
    # Independent motor-index oracle from the user's current tip-to-palm map.
    ticks=np.arange(2048,2064)
    expected=np.array([ticks[i-1]-2048 for i in (8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1)])*2*np.pi/4096
    assert np.array_equal(runtime.to_sim(ticks),expected);checks.append('current_motor_mapping')
    for seq in range(3):
        sample.update(sequence=seq,sample_time=seq*.075,pose_time=seq*.075)
        out=runtime.propose(sample)
        if seq<2:assert out['state']=='warming'
    assert runtime.phase==0 and runtime.pending is not None
    assert max(abs(np.array(out['goal_ticks'])-sample['commanded_ticks']))<=2
    checks.extend(['three_fresh_sample_warmup','maximum_two_tick_goal','phase_waits_for_acknowledgement'])
    rejected(lambda:runtime.propose(dict(sample,sequence=3,sample_time=.225,pose_time=.225)),'pending_goal_rejects_new_proposal')
    wrong=np.array(out['goal_ticks']);wrong[0]+=1
    rejected(lambda:runtime.commit_shadow(2,wrong),'changed_acknowledgement')
    assert runtime.phase==0
    before=sample['commanded_ticks'].copy();runtime.commit_shadow(2,out['goal_ticks'])
    assert runtime.phase==1
    actual=(np.array(out['goal_ticks'])[SIM_IDS-1]-before[SIM_IDS-1])*TICK/.2
    assert np.allclose(runtime.last_action,actual,atol=1e-12)
    checks.append('last_action_describes_executed_goal_change')
    fresh=dict(sample,sequence=3,sample_time=.225,pose_time=.225,commanded_ticks=np.array(out['goal_ticks']))
    rejected(lambda:runtime.propose(dict(fresh,pose_time=0.)),'stale_cube_pose')
    rejected(lambda:runtime.propose(dict(fresh,palm_point='hand_body_origin')),'wrong_palm_reference')
    rejected(lambda:runtime.propose(dict(fresh,frame_id='camera_optical_frame')),'unregistered_frame')
    rejected(lambda:runtime.propose(dict(fresh,sequence=2)),'duplicate_sample')
    rejected(lambda:runtime.propose(dict(fresh,sample_time=.2,pose_time=.2)),'too_short_control_interval')
    assert runtime.phase==1
    out=runtime.propose(fresh);runtime.commit_shadow(out['sequence'],out['goal_ticks'])
    assert runtime.phase==2;checks.append('valid_new_sample_resumes_shadow')
    # Test only the hold state transition, not a simulated movement success.
    runtime.phase=len(runtime.goals)
    hold=dict(fresh,sequence=4,sample_time=.3,pose_time=.3,commanded_ticks=np.array(out['goal_ticks']))
    out=runtime.propose(hold)
    assert out['state']=='shadow_hold' and out['next_rotation_ready'] is False
    assert out['motor_commands_sent']==0;checks.append('hold_keeps_proposing_without_auto_open_or_off')
    assert 'serial' not in sys.modules and 'dynamixel_sdk' not in sys.modules and 'mujoco' not in sys.modules
    result={'status':'passed','checks':checks,'devices_opened':0,'motor_commands_sent':0,
            'physical_camera_registration_verified':False,'hardware_execution_allowed':False,
            'state_transition_unit_checks_not_physical_rotation_success':True}
    (HERE/'shadow_runtime_verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
