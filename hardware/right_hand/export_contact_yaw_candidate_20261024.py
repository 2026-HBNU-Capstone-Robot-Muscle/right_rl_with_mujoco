"""Package the bounded exploratory contact candidate; never run hardware."""
import hashlib
import json
from pathlib import Path
import cv2 as cv
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'


def main():
    path=OUT/'contact_point_yaw_search.json';source=path.read_bytes();r=json.loads(source)
    assert r['status']=='contact_point_kinematic_and_active_cube_diagnostic'
    assert r['hardware_execution_allowed'] is False and r['marker_registration_verified'] is False
    candidates=r['promising_candidates'];assert len(candidates)==1
    choice=candidates[0];selected=choice['summary'];goals=np.asarray(choice['goal_ticks']);base=np.array(r['base_goal_ticks'])
    assert selected['kinematic_target_degrees']==-3. and selected['desired_physical_yaw_degrees']==3.
    assert selected['promising'] and selected['direction_consistent_in_evaluated_scenarios']
    assert selected['held_out_pass'] is False  # Preserve the small-angle shortfall.
    cases=selected['active_cube_replays']+selected['held_out_cases']
    assert len(cases)==9 and all(c['finite'] and c['yaw_about_gravity_up_deg']>0 for c in cases)
    assert goals.shape==(90,16) and np.isfinite(goals).all() and np.all(goals==np.rint(goals))
    assert np.max(abs(goals-base))<=64 and np.max(abs(np.diff(np.vstack([base,goals]),axis=0)))<=2
    frozen=np.array([4,8,12,13,14,15,16])-1
    assert np.all(goals[:,frozen]==base[frozen])
    assert max(abs(np.array(r['closure_delta_ticks'])))==18
    references=[r['source'],'right_hand_rotation_20261024/contact_point_yaw_search.json',
        'right_hand_rotation_20261024/completed_yaw_active_cube_replay.json',
        'slow_grasp_demo_20261023_182105_573825/on_paired_pose_review.json',
        'right_hand_rotation_20261023/native_motor_limits.json']
    record=dict(version='contact_refined_small_yaw_candidate_v1',base_goal_ticks=base.tolist(),goal_ticks=goals.tolist(),
        source_hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in references},
        source_trial=r['source'],source_baseline_phase='stage_4_fixed_grasp_observation',
        recorded_following_residual_ticks=r['recorded_following_residual_ticks'],
        closure_delta_ticks=r['closure_delta_ticks'],contact_refinement_samples=20,rotation_samples=70,
        selected=selected,desired_physical_yaw_degrees=3.,kinematic_angle_parameter_degrees=-3.,
        full_held_out_angle_criterion_passed=False,evaluated_direction_consistent=True,
        evaluated_simulated_yaw_range_deg=[min(c['yaw_about_gravity_up_deg'] for c in cases),max(c['yaw_about_gravity_up_deg'] for c in cases)],
        evaluated_simulated_translation_range_mm=[min(c['lateral_translation_mm'] for c in cases),max(c['lateral_translation_mm'] for c in cases)],
        axis_world_candidate=r['axis_world'],sample_period_s=.05,maximum_delta_limit_ticks=64,
        experimental_trial_only=True,robust_policy=False,cube_dynamic_contact_simulated=True,
        marker_to_model_registration_verified=False,cube_friction_measured=False,
        current_or_PID_changed=False,joint_offsets_fitted=False,live_sac=False,
        physical_cube_rotation_verified=False,hardware_execution_allowed=False,
        limitations=['All nine evaluated contact scenarios preserve the predicted positive direction, but one response-offset case rotates only about 0.93 degrees.',
            'Hand response is prescribed and the cube pose/friction are hypotheses; this is not a calibrated physical prediction.',
            'The kinematic angle parameter is negative; positive physical yaw is the contact-simulation prediction, not a sign relabeling of the kinematic target.',
            'An operator trial still requires a fresh full native path proof and unchanged health/communication stops.'])
    destination=OUT/'contact_refined_small_yaw_candidate.json'
    destination.write_text(json.dumps(record,indent=2),encoding='utf-8')
    print('CANDIDATE',destination,'SHA256',hashlib.sha256(destination.read_bytes()).hexdigest())
    print('MODEL SUMMARY',record['evaluated_simulated_yaw_range_deg'],record['evaluated_simulated_translation_range_mm'])
    frames_path=OUT/'contact_point_yaw_simulation_frames.npz'
    if frames_path.exists():
        with np.load(frames_path) as z:frames=z['frames'];fps=float(z['fps'])
        previous=Path.cwd()
        import os
        try:
            os.chdir(OUT);writer=cv.VideoWriter('contact_point_yaw_simulation.mp4',cv.VideoWriter_fourcc(*'mp4v'),fps,(640,480))
            assert writer.isOpened()
            try:
                for frame in frames:
                    frame=cv.cvtColor(frame,cv.COLOR_RGB2BGR)
                    cv.rectangle(frame,(0,0),(640,48),(0,0,0),-1)
                    cv.putText(frame,'CONTACT MODEL DIAGNOSTIC: 55mm / 100g',(8,19),cv.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
                    cv.putText(frame,'Prescribed hand response; physical rotation NOT verified',(8,39),cv.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
                    writer.write(frame)
            finally:writer.release()
            view=cv.cvtColor(frames[len(frames)//2],cv.COLOR_RGB2BGR)
            (OUT/'contact_point_yaw_simulation.jpg').write_bytes(cv.imencode('.jpg',view)[1].tobytes())
            decoder=cv.VideoCapture('contact_point_yaw_simulation.mp4')
            try:assert decoder.isOpened() and int(decoder.get(cv.CAP_PROP_FRAME_COUNT))==len(frames)
            finally:decoder.release()
        finally:os.chdir(previous)
        print('VIDEO',OUT/'contact_point_yaw_simulation.mp4','seconds',len(frames)/fps)


if __name__=='__main__':main()
