"""Position-target proposals account for model gravity along a handoff path.

This changes no model/PID/force limit. Gravity and contact-offset calculations
are model-only trajectory proposals, always checked by free native dynamics.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target
from handoff_support_wrench import contact_wrench


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=('gravity_source_bias','gravity_late_bias','lag_interpolation'),required=True)
    p.add_argument('--model-kind',choices=('active','active_ccd50'),default='active');a=p.parse_args()
    folder=OUT/'next_cycle/handoff_refinement_20261110'/(a.mode+('_'+a.model_kind if a.model_kind!='active' else ''));folder.mkdir(exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed gravity comparison')
    (folder/'source_replay_gravity_compensated_bridge.py').write_bytes(Path(__file__).read_bytes())
    source=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz'
    endpoint=OUT/'next_cycle/handoff_refinement_20261110/endpoint_screen/qualified3/end_state.npz'
    ref_path=OUT/'next_cycle/rrt_rf_seed5_integer_equilibrium3/research_goals.npz'
    z=dict(np.load(source,allow_pickle=False));end=dict(np.load(endpoint,allow_pickle=False));ref=np.load(ref_path,allow_pickle=False)['goals'][:-134]
    m,_,_,_,provenance=assets(a.model_kind);d=restore(m,z);guard=Guard(m);cube=m.geom('cube').id
    start=motor(d.ctrl);lag=z['ctrl']-z['qpos'][:16];end_lag=end['ctrl']-end['qpos'][:16]
    initial=restore(m,z);initial.qvel[:]=0;mujoco.mj_forward(m,initial)
    last=restore(m,end);last.qvel[:]=0;mujoco.mj_forward(m,last)
    gain=m.actuator_gainprm[:,0]
    source_extra=lag-initial.qfrc_bias[:16]/gain;end_extra=end_lag-last.qfrc_bias[:16]/gain
    query=restore(m,z);corrected=[];goal_before=start.copy();clipped=[]
    for i,goal in enumerate(ref):
        q=z['qpos'][:16]+ticks_to_sim(goal)-ticks_to_sim(start)
        alpha=float(np.clip((i/max(1,len(ref)-1)-.6)/.4,0,1));alpha=alpha*alpha*(3-2*alpha)
        query.qpos[:]=z['qpos'];query.qpos[:16]=q;query.qvel[:]=0;mujoco.mj_forward(m,query)
        if a.mode=='lag_interpolation':offset=lag*(1-alpha)+end_lag*alpha
        else:
            extra=source_extra if a.mode=='gravity_source_bias' else source_extra*(1-alpha)+end_extra*alpha
            offset=query.qfrc_bias[:16]/gain+extra
        proposed=motor(q+offset)
        # Preserve the index-base support command from the proven first grip.
        proposed[7]=start[7]
        legal=np.clip(proposed,np.ceil(guard.lo).astype(int),np.floor(guard.hi).astype(int))
        if np.any(legal!=proposed):clipped.append(i)
        steps=max(1,int(np.ceil(np.max(abs(legal-goal_before))/2)))
        for k in range(1,steps+1):corrected.append(np.rint(goal_before+(legal-goal_before)*k/steps).astype(int))
        goal_before=legal.copy()
    corrected.extend(goal_before.copy() for _ in range(200));goals=np.asarray(corrected)
    if np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))>2:raise AssertionError('Goal spacing')
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();tail=good=0;stop=None;first_B=first_C=None;drift=shift=0.;began=time.perf_counter()
    for i,goal in enumerate(goals):
        d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=len(goals)-54:
                w=contact_wrench(m,d,guard.hand);tail+=1
                good+=all(w['by_geom'].get(n,{}).get('normal_N',0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1'))
                B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
                if first_B is None:first_B=B.copy();first_C=C.copy()
                drift=max(drift,orientation_error_deg(B,first_B));shift=max(shift,float(np.linalg.norm(C-first_C)*1000))
            if stop:break
        if (i+1)%400==0 or stop:print(json.dumps(dict(mode=a.mode,goals=i+1,stop=stop)),flush=True)
        if stop:break
    B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    angle=orientation_error_deg(B,target);center=float(np.linalg.norm(C-station)*1000);w=contact_wrench(m,d,guard.hand)
    passed=not stop and tail==4050 and good/tail>.8 and angle<5 and center<2 and drift<3 and shift<2
    np.savez_compressed(folder/'research_goals.npz',start=start,goals=goals[:i+1])
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    r=dict(status='model_gravity_position_target_handoff_comparison',mode=a.mode,model_provenance=provenance,
        source_state=str(source.relative_to(OUT)),source_sha256=sha(source),reference_path=str(ref_path.relative_to(OUT)),reference_sha256=sha(ref_path),
        endpoint_path=str(endpoint.relative_to(OUT)),endpoint_sha256=sha(endpoint),goals_completed=i+1,planned_goals=len(goals),stop=stop,
        temporary_RF_handoff_and_pose_passed=bool(passed),target90_error_deg=angle,common_station_error_mm=center,
        relative_orientation_error_deg=orientation_error_deg(B,B0),relative_center_error_mm=float(np.linalg.norm(C-C0)*1000),
        final_contact_forces_N_model_only={name:r['normal_N'] for name,r in w['by_geom'].items()},
        tail_1ms_samples=tail,RF_TH_base_support_fraction=good/max(1,tail),hold_orientation_drift_deg=drift,hold_center_drift_mm=shift,
        worst=guard.worst,admissible_native_range_projection_count=len(clipped),
        actual_cube_not_reset_after_source=True,model_PID_force_and_collision_settings_changed=False,
        model_gravity_is_not_calibrated_physical_torque=True,next90_rotation_passed=False,
        model_kind=a.model_kind,source_state_transferred_from_old_numerical_model=a.model_kind!='active',
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
