"""Preserve successes AND failures; finalize a nominal supervised-trial scope."""
import hashlib,json,datetime as dt
from pathlib import Path
from pitch_slow_yaw_plan_20261029 import ROOT,OUT,LOADED_PATH,REQUEST,PROOF,PitchSlowYawPlan,hashes

DATA=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic'
def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
    cases=[];sources=[]
    for front in (-15,-10,-5):
        path=DATA/f'whole_parameters_front{front}/review.json';r=read(path)
        assert r['path_sha256']==sha(LOADED_PATH)
        assert r['execution_script_snapshot_captured_at_start'] is True
        snapshot=path.parent/'simulation_runner_snapshot.py.txt';assert sha(snapshot)==r['simulation_script_sha256']
        assert r['cube_pose_prescribed_during_rollout'] is False and r['hand_pose_prescribed_during_rollout'] is False
        assert len(r['cases'])==9
        cases.extend(r['cases']);sources.extend(dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p)) for p in (path,snapshot))
    pose_path=DATA/'yaw150_pose9_candidate/review.json';poses=read(pose_path)
    nominal=[r for r in cases if (r['gain_scale'],r['friction'])==(1.,.3)]
    assert len(nominal)==3 and all(r['motion_health_passed'] for r in nominal)
    assert len(poses['cases'])==9 and all(r['motion_health_passed'] for r in poses['cases'])
    sources.append(dict(path=pose_path.relative_to(ROOT).as_posix(),sha256=sha(pose_path)))
    seed=OUT/'camera_pitch_seed_hypothesis.json'
    sources.append(dict(path=seed.relative_to(ROOT).as_posix(),sha256=sha(seed)))
    evidence=dict(status='nominal_continuous_model_supervised_experiment_scope',accepted_for_supervised_combined_trial=True,case_count=27,cases=cases,
        nominal_case_health_passed=True,nominal_cases_passed=3,all_case_health_passed=all(r['motion_health_passed'] for r in cases),
        broad_parameter_cases_passed=sum(r['motion_health_passed'] for r in cases),
        wide_parameter_robustness_verified=False,supervised_experiment_only=True,
        nominal_pose_sensitivity_cases=poses['cases'],loaded_path_sha256=sha(LOADED_PATH),bound_sources=sources,
        cube_initialized_once=True,cube_pose_prescribed_during_rollout=False,
        initial_cube_metric_registration_verified=False,physical_combined_rotation_verified=False,
        admission_rationale='One user-supervised experiment of previously demonstrated primitives. Nominal full cube trajectory and all hand-route conditions pass; uncalibrated broad gain/friction failures are retained and explicitly not certified.',
        motor_joint_and_health_stops_unchanged=True,separate_combined_macro_walltime_limit_s=480,
        original_angular_trajectories_unchanged=True,yaw_goal_cadence_changed_to150ms=True,assistant_motor_writes=0)
    (OUT/'loaded_cube_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    from pitch_slow_yaw_gate_20261029 import make_review,validate_review
    plan=PitchSlowYawPlan(REQUEST);before=hashes();empty=read(OUT/'empty_recovery_screen/review.json')
    review=make_review(plan,empty,before)
    PROOF.write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    validate_review(plan,PROOF)
    print(json.dumps(dict(status='nominal_supervised_trial_review_created',review=str(PROOF),
        nominal_full_cases=3,broad_parameter_passes=evidence['broad_parameter_cases_passed'],broad_parameter_cases=27,
        pose_neighborhood_cases=9,empty_conditions=210,full_hand_dynamic_cases=9,
        exact90_degrees_verified=False,hardware_writes=0)))
if __name__=='__main__':main()
