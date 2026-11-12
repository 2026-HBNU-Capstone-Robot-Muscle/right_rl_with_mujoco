"""Replay model observer fixtures through a packaged controller, never devices."""
from pathlib import Path
import argparse
import hashlib
import importlib
import json
import sys
import os
for _cycle_thread_env in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_cycle_thread_env]='1'
import numpy as np

OUT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',default='next_cycle/handoff_refinement_20261110/robust_cycle_preparation_20261111/encoder_pose_feedback_v1_complete_bundle/review.json');p.add_argument('--case');a=p.parse_args()
    report_path=(OUT/a.trial).resolve()
    if not report_path.is_relative_to(OUT.resolve()):raise ValueError('Local research report required')
    prior=json.loads(report_path.read_text(encoding='utf-8'));bundle=OUT/prior['bundle'];assert sha(bundle/'bundle.json')==prior['bundle_metadata_sha256']
    meta=json.loads((bundle/'bundle.json').read_text(encoding='utf-8'));sys.path.insert(0,str(bundle))
    if meta.get('pose_feedback_is_not_SAC'):cls=importlib.import_module('pose_residual_cycle_runtime').PoseResidualCycleController
    elif meta.get('clearance_selection_method'):cls=importlib.import_module('contact_trajectory_cycle_runtime').TrajectoryClearanceCycleController
    else:cls=importlib.import_module('whole_cycle_shadow_runtime').WholeCycleShadowController
    results=[]
    for row in prior['rows']:
        if a.case and row['case']!=a.case:continue
        fixture_path=OUT/row['path']/'observer_fixture.npz';f=np.load(fixture_path,allow_pickle=False);c=cls(bundle,observation_mode='simulator_fixture');c.meta['phase_speed_mode']=row['phase_speed_mode'];c.meta['MF_static_margin_mm']=row['MF_static_margin_mm'];comparisons=0;warm=0;last=None
        for index,seq in enumerate(f['sequence']):
            sample={name:f[name][index] for name in ('measured_ticks','commanded_ticks','palm_position','cube_position','cube_quaternion')};sample.update(sequence=int(seq),sample_time=float(f['sample_time'][index]),pose_time=float(f['pose_time'][index]),frame_id='leap_sim_world',palm_point='policy_palm_position',observation_origin='simulator_fixture')
            proposed=c.propose(sample);expected=f['expected_goal'][index]
            if np.isnan(expected).all():assert proposed['state']=='warming';warm+=1;continue
            assert np.array_equal(expected,proposed['goal_ticks']),(row['case'],index,expected,proposed['goal_ticks'])
            assert proposed['motor_commands_sent']==0 and not proposed['hardware_execution_allowed'];c.commit_shadow(int(seq),expected);comparisons+=1;last=sample
        checks=[]
        def rejects(call):
            try:call()
            except ValueError:return
            raise AssertionError('Invalid shadow input/acknowledgement was accepted')
        rejects(lambda:c.propose(last));checks.append('Duplicate input sequence rejected')
        next_sample=dict(last,sequence=int(last['sequence'])+1,sample_time=float(last['sample_time'])+.075,pose_time=float(last['sample_time'])+.075,commanded_ticks=c.last_goal.copy())
        rejects(lambda:c.propose(dict(next_sample,pose_time=next_sample['sample_time']-.151)));checks.append('Stale cube pose rejected')
        rejects(lambda:c.propose(dict(next_sample,frame_id='camera')));checks.append('Unregistered frame rejected')
        rejects(lambda:c.propose(dict(next_sample,commanded_ticks=c.last_goal+1)));checks.append('Changed acknowledged command rejected')
        registered=cls(bundle)
        first={name:f[name][0] for name in ('measured_ticks','commanded_ticks','palm_position','cube_position','cube_quaternion')};first.update(sequence=0,sample_time=0.,pose_time=0.,frame_id='leap_sim_world',palm_point='policy_palm_position',registration=dict(verified=True,registration_id='unverified-placeholder'),bus_checks_passed=True)
        rejects(lambda:registered.propose(first));assert registered.phase==0 and registered.pending is None;checks.append('Default registered-observer mode refuses current unverified physical registration')
        if c.phase>=5089:
            result=c.propose(next_sample);assert np.array_equal(result['goal_ticks'],c.last_goal);rejects(lambda:c.propose(next_sample));rejects(lambda:c.commit_shadow(result['sequence'],np.asarray(result['goal_ticks'])+1));c.commit_shadow(result['sequence'],result['goal_ticks']);assert c.phase==5089
            done=c.completion();assert done['final_goal_is_kept'] and not done['automatic_open'] and not done['automatic_torque_off'] and not done['next_rotation_or_left_hand_authorized'];checks.append('Final command is retained; duplicate proposal/changed acknowledgement fail; no automatic OPEN/OFF or left-hand authorization')
        result=dict(case=row['case'],fixture=fixture_path.relative_to(OUT).as_posix(),fixture_sha256=sha(fixture_path),goal_comparisons=comparisons,warming_samples=warm,exact_mismatches=0,final_reference_phase=c.phase,checks=checks,native_task_passed=row['two_turn_reached_location_holds_passed']);results.append(result)
        print(json.dumps(dict(case=row['case'],goal_comparisons=comparisons,exact_mismatches=0)),flush=True)
    output=report_path.parent/('shadow_fixture_verification'+('_'+a.case if a.case else '')+'.json');verification=dict(status='passed',trial=report_path.relative_to(OUT).as_posix(),trial_sha256=sha(report_path),bundle_metadata_sha256=sha(bundle/'bundle.json'),cases=results,goal_comparisons=sum(r['goal_comparisons'] for r in results),motor_commands_sent=0,physical_registration_verified=False,verification_is_no_output_runtime_parity_not_hardware_success=True,devices_opened=0,hardware_execution_allowed=False)
    output.write_text(json.dumps(verification,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(dict(status='passed',cases=len(results),goal_comparisons=verification['goal_comparisons'],devices_opened=0,hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
