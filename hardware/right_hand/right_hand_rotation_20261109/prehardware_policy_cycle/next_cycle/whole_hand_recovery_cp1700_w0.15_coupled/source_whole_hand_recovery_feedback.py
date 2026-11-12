"""Local model feedback at an actual supported CCD50 rollout checkpoint.

Every proposed goal is predicted in free native dynamics. Only a persistent
actual model state executes the chosen goals. This teacher probe is not SAC,
distillation, a physical test, or proof of the whole two-turn task.
"""
import argparse
from pathlib import Path
import time
import json
import re
from collections import Counter
import numpy as np
import mujoco
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,BOUND,save_json,sha
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from whole_hand_support import sample_model,CONTRACT


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--source-index',type=int,default=1700)
    parser.add_argument('--steps',type=int,default=240);parser.add_argument('--center-weight',type=float,default=.15)
    parser.add_argument('--tag',default='');parser.add_argument('--coupled-fallback',action='store_true')
    args=parser.parse_args()
    if not 1<=args.steps<=600 or not .02<=args.center_weight<=1.:raise ValueError('Bounded research workload required')
    if args.tag and not re.fullmatch('[a-z0-9_]+',args.tag):raise ValueError('Use a simple research tag')
    folder=OUT/(f'next_cycle/whole_hand_recovery_cp{args.source_index}_w{args.center_weight:g}'+('_'+args.tag if args.tag else ''))
    if folder.exists():raise RuntimeError('Preserve previous feedback experiment')
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool_review=json.loads((pool_path.parent/'review.json').read_text())
    if sha(pool_path)!=pool_review['state_pool_sha256']:raise RuntimeError('State pool changed')
    pool=np.load(pool_path,allow_pickle=False)
    position=np.flatnonzero(pool['indices']==args.source_index)
    if len(position)!=1:raise ValueError('Requested checkpoint not in actual rollout pool')
    old_path=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/research_trace.npz'
    whole_path=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_whole_hand_v1_active_ccd50/research_trace.npz'
    old_goals=np.load(old_path,allow_pickle=False)['goals'];whole_goals=np.load(whole_path,allow_pickle=False)['goals']
    if not np.array_equal(old_goals,whole_goals):raise RuntimeError('Whole-hand accounting changed executed parent goals')
    m,initial,_,_,provenance=assets('active_ccd50');z=dict(initial);index=int(position[0])
    z.update({key:pool[key][index].copy() for key in BOUND});z['time']=float(pool['time'][index])
    d=restore(m,z);checker=Guard(m);cube=m.geom('cube').id;geometry=precompute(m,checker.hand)
    target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90)
    center=d.geom_xpos[cube].copy();mids=np.array(SIM_TO_MOTOR_IDS)-1
    folder.mkdir();(folder/'source_whole_hand_recovery_feedback.py').write_bytes(Path(__file__).read_bytes())
    np.savez_compressed(folder/'start_state.npz',**{key:getattr(d,key).copy() for key in BOUND},time=d.time,
                        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    began=time.perf_counter();rows=[];trace=[];best_score=float('inf');stale=held=0;remainder=np.zeros(16)
    rejections=Counter();coupled_fallback_events=[]

    def evidence(data):
        B=data.geom_xmat[cube].reshape(3,3);C=data.geom_xpos[cube]
        rotation=np.degrees(Rotation.from_matrix(target@B.T).as_rotvec())
        error=orientation_error_deg(B,target)
        support=sample_model(m,data,checker.hand,cube)
        gap=min(value for _,value in gaps(m,data,C,B,geometry,True))
        load_deficit=max(0.,support['weight_N']-support['upward_support_N'])*3.
        gap_penalty=max(0.,.5-gap)*2. if error<10 else 0.
        residual=np.r_[rotation,(C-center)*1000*args.center_weight,load_deficit,gap_penalty]
        return residual,float(np.linalg.norm(residual)),dict(
            error_deg=error,tilt_deg=face_up_error_deg(B,5,initial['up']),
            center_drift_from_checkpoint_mm=float(np.linalg.norm(C-center)*1000),
            upper_screen_mm=gap,whole_hand_support=support)

    def predict(goal):
        if np.any(goal<checker.lo) or np.any(goal>checker.hi):rejections['native_goal_range']+=1;return None
        state={key:getattr(d,key).copy() for key in BOUND};state['time']=d.time
        trial=restore(m,state);trial.ctrl[:]=ticks_to_sim(goal);guard=Guard(m)
        guard.gap=checker.gap;guard.longest_gap=checker.longest_gap
        for _ in range(75):
            mujoco.mj_step(m,trial)
            reasons=guard.check(trial)
            if reasons:rejections.update(reasons);return None
            if np.linalg.norm(trial.qpos[-7:-4]-initial['initial_cube_center'])>.08:
                rejections['cube_translation_over80mm']+=1;return None
        return trial,evidence(trial)

    start=evidence(d)[2];stop=None
    for step in range(args.steps):
        previous=motor(d.ctrl);zero=predict(previous)
        base=zero[1][0] if zero else evidence(d)[0]
        candidates=[(previous,zero)] if zero else [];J=np.zeros((len(base),16))
        for joint in range(16):
            ends=[]
            for sign in (-1,1):
                goal=previous.copy();goal[mids[joint]]+=sign*2;trial=predict(goal)
                if trial is not None:candidates.append((goal,trial));ends.append(trial[1][0])
                else:ends.append(None)
            if all(end is not None for end in ends):J[:,joint]=(ends[1]-ends[0])/4
            elif ends[0] is not None:J[:,joint]=(base-ends[0])/2
            elif ends[1] is not None:J[:,joint]=(ends[1]-base)/2
        fit=lsq_linear(np.vstack([J,.02*np.eye(16)]),np.r_[-.35*base,np.zeros(16)],bounds=(-2,2),max_iter=100)
        intended=fit.x+remainder;delta=np.clip(np.rint(intended).astype(int),-2,2)
        remainder=np.clip(intended-delta,-.4999999,.4999999)
        for scale in (1.,.5):
            goal=previous.copy();goal[mids]+=np.rint(delta*scale).astype(int);trial=predict(goal)
            if trial is not None:candidates.append((goal,trial))
        if not candidates and args.coupled_fallback:
            random=np.random.default_rng(args.source_index*1000+step)
            for _ in range(64):
                goal=previous+random.choice([-2,0,2],16);trial=predict(goal)
                if trial is not None:candidates.append((goal,trial))
            coupled_fallback_events.append(dict(after_goals=len(rows),tested=64,valid=len(candidates)))
        if not candidates:stop='no_safe_two_tick_proposal';break
        goal,prediction=min(candidates,key=lambda pair:pair[1][1][1])
        if np.max(abs(goal-previous))>2:raise AssertionError('Rate contract changed')
        d.ctrl[:]=ticks_to_sim(goal);rows.append(goal.copy())
        for _ in range(75):
            mujoco.mj_step(m,d)
            if checker.check(d):stop='native_dynamic_guard';break
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop='cube_translation_over80mm';break
        prediction_error=float(np.max(abs(d.qpos-prediction[0].qpos)))
        _,score,actual=evidence(d);actual.update(step=step+1,prediction_qpos_error=prediction_error)
        trace.append(actual)
        if prediction_error>1e-8:stop='prediction_state_alignment';break
        good=(actual['error_deg']<5 and actual['tilt_deg']<5 and actual['center_drift_from_checkpoint_mm']<2 and
              actual['upper_screen_mm']>.3 and actual['whole_hand_support']['supported_sample'])
        held=held+1 if good else 0
        stale=stale+1 if score>=best_score-.001 else 0;best_score=min(best_score,score)
        if (step+1)%20==0:
            save_json(folder/'progress.json',dict(step=step+1,actual=actual,elapsed_seconds=time.perf_counter()-began))
            print(json.dumps(dict(step=step+1,error_deg=actual['error_deg'],drift_mm=actual['center_drift_from_checkpoint_mm'],
                upward_N=actual['whole_hand_support']['upward_support_N'],contacts=actual['whole_hand_support']['contact_hand_geoms'])),flush=True)
        if stop or held>=54:break
        if stale>=30 and not good:stop='local_model_search_stalled';break
    np.savez_compressed(folder/'research_goals.npz',start=motor(z['ctrl']),goals=np.asarray(rows))
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=initial['up'],initial_cube_center=initial['initial_cube_center'],initial_cube_rotation=initial['initial_cube_rotation'])
    result=dict(status='whole_hand_native_local_feedback_probe',source_index=args.source_index,
        source_state_pool_sha256=sha(pool_path),parent_goals_sha256=sha(old_path),whole_hand_parent_goals_identical=True,
        center_optimization_weight=args.center_weight,steps=len(rows),stop=stop,start=start,
        source_sha256=sha(Path(__file__)),coupled_fallback_enabled=args.coupled_fallback,
        coupled_fallback_events=coupled_fallback_events,prediction_rejections=dict(rejections),
        final=trace[-1] if trace else start,best_orientation_error_deg=min([start['error_deg']]+[row['error_deg'] for row in trace]),
        consecutive_qualifying_control_samples=held,local_target_screen_held=held>=54,
        trace=trace,worst=checker.worst,model_provenance=provenance,support_contract=CONTRACT,
        support_source_sha256=sha(Path(__file__).with_name('whole_hand_support.py')),
        current_center_is_not_final_common_station=True,
        upper_screen_is_not_a_strict_clearance_certificate=True,
        initialized_checkpoint_branch_not_full_initial_placement_success=True,
        model_feedback_teacher_is_not_SAC=True,first_rotation_and_hold_certified=False,
        complete_cycle_passed=False,devices_opened=0,hardware_execution_allowed=False,
        elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',result)
    print(json.dumps({key:result[key] for key in ('status','steps','stop','best_orientation_error_deg','local_target_screen_held','hardware_execution_allowed')}),flush=True)


if __name__=='__main__':main()
