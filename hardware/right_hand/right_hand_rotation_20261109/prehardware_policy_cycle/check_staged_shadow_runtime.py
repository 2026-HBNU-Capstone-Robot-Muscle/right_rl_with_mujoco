"""Model-trace parity and transactional rejection checks, with no device APIs."""
from pathlib import Path
import json
import copy
import hashlib
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from staged_shadow_runtime import StagedShadowController
from shadow_runtime import multiply,matrix,SIM_IDS,TICK

OUT=Path(__file__).resolve().parent
BUNDLE=OUT/'next_cycle/staged_shadow_bundle_v2'


def quat_from_matrix(R):
    # Symmetric eigenproblem avoids a fragile trace-only conversion near180deg.
    x,y,z=R[0];u,v,w=R[1];a,b,c=R[2]
    K=np.array([[x-v-c,y+u,z+a,b-w],
                [y+u,v-x-c,w+b,z-a],
                [z+a,w+b,c-x-v,u-y],
                [b-w,z-a,u-y,x+v+c]])/3
    q=np.linalg.eigh(K)[1][:,-1][[3,0,1,2]]
    return q if q[0]>=0 else -q


def motor_float(q):
    ticks=np.empty(16);ticks[SIM_IDS-1]=np.asarray(q)/TICK+2048
    return ticks


def fixture_sample(controller,obs,command,sequence,gap=1.):
    r1=np.asarray(obs[73:76],float);r2=np.asarray(obs[76:79],float)
    R=np.stack([np.cross(r1,r2),r1,r2])
    u,_,v=np.linalg.svd(R);R=u@v
    q=quat_from_matrix(R@matrix(controller.goal_quat))
    palm=np.array([.11,0,.03])
    return dict(sequence=int(sequence),sample_time=sequence*.075,pose_time=sequence*.075,
                frame_id='leap_sim_world',palm_point='policy_palm_position',
                measured_ticks=motor_float(obs[:16]).tolist(),commanded_ticks=np.asarray(command).tolist(),
                palm_position=palm.tolist(),cube_position=(palm-obs[64:67]).tolist(),cube_quaternion=q.tolist(),
                upper_sweep_acquisition=dict(model_sha256=controller.meta['model_sha256'],sequence=int(sequence),screen_mm=float(gap),method='slab_clipped_radial_screen'))


def expect_rejection(call,label):
    try:call()
    except ValueError:return label
    raise AssertionError('Failed to reject: '+label)


def snapshot(c):
    return (c.phase,c.mode,c.acquired,c.correction_initialized,c.sequence,c.last_time,
            c.joint_history.copy(),c.position_history.copy(),c.orientation_history.copy(),c.last_action.copy())


def same_snapshot(a,b):
    return all(np.array_equal(x,y) for x,y in zip(a,b))


def main():
    cases=[];checks=[];selected=OUT/'sac_current_contract_v4_20k/export_000000010256'
    student=OUT/'next_cycle/recovery_bc_v2_all_examples'
    for case in ('nominal','x_plus05','friction025'):
        c=StagedShadowController(BUNDLE)
        parent='evaluation_active_certified' if case=='nominal' else 'evaluation_active_'+case
        prefix=dict(np.load(selected/parent/'closed_loop_trace.npz',allow_pickle=False))
        suffix=dict(np.load(student/('evaluation_'+case+'_latched_hold_acq8_fixed_position_hold')/'research_trace.npz',allow_pickle=False))
        review=json.loads((student/('evaluation_'+case+'_latched_hold_acq8_fixed_position_hold')/'review.json').read_text())
        initial=dict(np.load(OUT.parents[1]/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/initial_state.npz',allow_pickle=False))
        command=c.to_motor(initial['ctrl']);sequence=0;mode_switches=[];max_action_error=0.;mismatches=[]
        for _ in range(2):
            value=c.propose(fixture_sample(c,prefix['observations'][0],command,sequence));sequence+=1
            assert value['state']=='warming' and c.phase==0
        sources=[('prefix',prefix,None),('suffix',suffix,review)]
        for label,source,r in sources:
            for i,(obs,expected) in enumerate(zip(source['observations'],source['goals'])):
                gap=1. if r is None or i==0 else r['trace'][i-1]['upper_gap_mm']
                sample=fixture_sample(c,obs,command,sequence,gap)
                previous_stage=c.mode;value=c.propose(sample)
                if label=='suffix' and i==0:
                    assert c.mode=='rotation' and not c.correction_initialized
                proposed=np.array(value['goal_ticks'])
                if not np.array_equal(proposed,expected):mismatches.append(dict(index=c.phase,stage=value['stage'],maximum_ticks=int(np.max(abs(proposed-expected)))))
                error=float(np.max(abs(np.array(value['actor_action'])-source['actions'][i])))
                max_action_error=max(max_action_error,error)
                c.commit_shadow(sequence,proposed);sequence+=1;command=proposed
                if c.mode!=previous_stage:mode_switches.append(dict(index=c.phase-1,stage=c.mode))
        if mismatches:raise AssertionError(dict(case=case,mismatches=mismatches[:10],count=len(mismatches)))
        assert c.mode=='hold' and c.phase==6556
        assert mode_switches==[dict(index=6300,stage='correction'),dict(index=6300+review['acquisition_step'],stage='hold')]
        capability=c.next_rotation_capability();assert not capability['ready'] and not capability['changes_hold_command']
        cases.append(dict(case=case,goals=c.phase,mismatched_goals=len(mismatches),maximum_action_error=max_action_error,stage_switches=mode_switches,
                          fractional_model_encoder_fixture=True,physical_sensor_accuracy_validated=False))
        # Failures must leave the committed mode and history unchanged.
        good=fixture_sample(c,suffix['observations'][-1],command,sequence)
        baseline=snapshot(c)
        for label,changes in [('missing_screen',{'upper_sweep_acquisition':{}}),
                              ('old_screen_sequence',{'upper_sweep_acquisition':dict(good['upper_sweep_acquisition'],sequence=sequence-1)}),
                              ('wrong_model_screen',{'upper_sweep_acquisition':dict(good['upper_sweep_acquisition'],model_sha256='wrong')}),
                              ('nonfinite_screen',{'upper_sweep_acquisition':dict(good['upper_sweep_acquisition'],screen_mm=float('nan'))}),
                              ('wrong_screen_method',{'upper_sweep_acquisition':dict(good['upper_sweep_acquisition'],method='unverified_distance')}),
                              ('stale_pose',{'pose_time':good['sample_time']-.151}),
                              ('ack_changed_command',{'commanded_ticks':(command+1).tolist()})]:
            bad=dict(good,**changes);checks.append(expect_rejection(lambda:c.propose(bad),case+':'+label));assert same_snapshot(baseline,snapshot(c))
        value=c.propose(good);pending_stage=c.mode
        checks.append(expect_rejection(lambda:c.commit_shadow(sequence,np.array(value['goal_ticks'])+1),case+':ack_changed_pending_goal'))
        assert c.mode==pending_stage and c.phase==6556
        c.commit_shadow(sequence,value['goal_ticks']);assert c.mode=='hold' and np.array_equal(c.last_goal,command)
        checks.append(case+':hold_commit_is_constant_and_does_not_enable_next_turn')
    result=dict(status='passed',cases=cases,transaction_checks=checks,
                source_runtime_sha256=hashlib.sha256((OUT/'staged_shadow_runtime.py').read_bytes()).hexdigest(),
                bundle_metadata_sha256=hashlib.sha256((BUNDLE/'bundle.json').read_bytes()).hexdigest(),
                model_fixture_goal_comparisons=sum(r['goals'] for r in cases),
                source_is_saved_perfect_model_sensor_contract_not_physical_sensor_data=True,
                physical_camera_registration_verified=False,complete_cycle_passed=False,devices_opened=0,hardware_execution_allowed=False)
    (OUT/'next_cycle/staged_shadow_runtime_verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
