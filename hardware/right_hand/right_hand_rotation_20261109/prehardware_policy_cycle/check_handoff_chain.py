"""Fresh first-turn controller plus preserved research suffix, with no resets.

The first segment uses fresh SAC/BC inference; subsequent teacher goals are
replayed solely to validate physical continuity, not as a learned cycle claim.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha
from staged_shadow_runtime import AdmissibleShadowController
from shadow_runtime import SIM_IDS,TICK
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from handoff_support_wrench import contact_wrench


def main():
    p=argparse.ArgumentParser();p.add_argument('--through',choices=('v6','v7'),default='v7');a=p.parse_args()
    names=['rrt_rf_seed5_integer_equilibrium3','handoff_integer_endpoint_close_v3',
           'handoff_integer_endpoint_close_v4','handoff_integer_endpoint_close_v5','handoff_integer_load_first_v6']
    if a.through=='v7':names+=['handoff_integer_load_first_v7']
    stages=[]
    for name in names:
        folder=OUT/'next_cycle'/name
        if not (folder/'review.json').exists():raise RuntimeError('Wait for completed report: '+name)
        stages.append((name,folder,dict(np.load(folder/'research_goals.npz',allow_pickle=False))))
    target_folder=OUT/'next_cycle/handoff_refinement_20261110'/('connected_chain_through_'+a.through)
    target_folder.mkdir(exist_ok=True)
    if (target_folder/'review.json').exists():raise RuntimeError('Preserve completed chain audit')
    (target_folder/'source_check_handoff_chain.py').write_bytes(Path(__file__).read_bytes())
    m,z,_,_,provenance=assets('active');d=restore(m,z);guard=Guard(m);cube=m.geom('cube').id
    c=AdmissibleShadowController(OUT/'next_cycle/staged_shadow_bundle_v2');geometry=precompute(m,guard.hand)
    reference=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation'
    first=np.load(reference/'research_trace.npz',allow_pickle=False)['goals'];sequence=0;goals=[];modes=[];seams=[];stop=None
    began=time.perf_counter();prefix_mismatches=0;tail_forces=[];tail_B=[];tail_C=[]

    def sample():
        measured=np.empty(16);measured[SIM_IDS-1]=np.rint(d.qpos[:16]/TICK+2048)
        q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[cube])
        s=dict(sequence=sequence,sample_time=sequence*.075,pose_time=sequence*.075,
            frame_id='leap_sim_world',palm_point='policy_palm_position',measured_ticks=measured,
            commanded_ticks=motor(d.ctrl),palm_position=np.array([.11,0.,.03]),cube_position=d.geom_xpos[cube].copy(),cube_quaternion=q)
        if c.phase>=c.prefix_steps:
            s['upper_sweep_acquisition']=dict(model_sha256=c.meta['model_sha256'],sequence=sequence,
                method='slab_clipped_radial_screen',screen_mm=min(v for _,v in gaps(m,d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3),geometry,True)))
        return s

    def execute(goal,mode,collect=False):
        nonlocal stop
        if np.max(abs(goal-motor(d.ctrl)))>2:stop=['two_tick_goal_spacing'];return
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):stop=['native_goal_range'];return
        d.ctrl[:]=ticks_to_sim(goal);goals.append(goal.copy());modes.append(mode)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if collect:
                w=contact_wrench(m,d,guard.hand);tail_forces.append(all(w['by_geom'].get(n,{}).get('normal_N',0.)>.2 for n in ('rf_tip','th_tip','if_bs_collision_1')))
                tail_B.append(d.geom_xmat[cube].reshape(3,3).copy());tail_C.append(d.geom_xpos[cube].copy())
            if stop:break

    for _ in range(2):assert c.propose(sample())['state']=='warming';sequence+=1
    for i in range(len(first)):
        proposal=c.propose(sample());goal=np.array(proposal['goal_ticks']);prefix_mismatches+=not np.array_equal(goal,first[i])
        c.commit_shadow(sequence,goal);sequence+=1;execute(goal,'live_first_turn_'+c.mode)
        if (i+1)%1000==0:print(json.dumps(dict(first_turn_live_goals=i+1,mismatches=prefix_mismatches,stop=stop)),flush=True)
        if stop:break
    def verify_seam(name,path):
        nonlocal stop
        saved=dict(np.load(path,allow_pickle=False))
        difference={key:float(np.max(abs(getattr(d,key)-saved[key]))) if saved[key].size else 0. for key in BOUND}
        seams.append(dict(stage=name,source_sha256=sha(path),max_abs=difference))
        if max(difference.values())>1e-8:stop=['persistent_state_seam_mismatch']
    if not stop:verify_seam('first_turn_end',reference/'end_state.npz')
    if prefix_mismatches:stop=['first_turn_goal_mismatch']
    for index,(name,folder,recorded) in enumerate(stages):
        if stop:break
        if not np.array_equal(recorded['start'],motor(d.ctrl)):stop=['suffix_start_command_mismatch'];break
        for i,goal in enumerate(recorded['goals']):
            execute(goal,'saved_teacher_'+name,collect=index==len(stages)-1 and i>=max(0,len(recorded['goals'])-54))
            if stop:break
        if not stop:verify_seam(name,folder/'end_state.npz')
        print(json.dumps(dict(completed_segment=name,total_goals=len(goals),stop=stop,seam=seams[-1])),flush=True)
    B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
    target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
    station=np.load(OUT/'baseline_archived_zero/end_state.npz',allow_pickle=False)['qpos'][-7:-4]
    pose_error=orientation_error_deg(B,target);center=float(np.linalg.norm(C-station)*1000)
    tail_count=len(tail_forces);fraction=sum(tail_forces)/max(1,tail_count)
    contiguous=not stop and len(seams)==len(stages)+1 and prefix_mismatches==0
    np.savez_compressed(target_folder/'research_trace.npz',goals=goals,modes=modes)
    np.savez_compressed(target_folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    report=dict(status='fresh_controller_and_preserved_handoff_suffix_chain',model_provenance=provenance,
        through=a.through,goals=len(goals),stop=stop,prefix_live_controller_goals=min(len(goals),len(first)),
        prefix_goal_mismatches=prefix_mismatches,seams=seams,continuous_dynamics_reproduction_passed=contiguous,
        first_to_handoff_target90_error_deg=pose_error,common_station_error_mm=center,
        final_up_tilt_deg=face_up_error_deg(B,5,z['up']),tail_1ms_samples=tail_count,rf_th_base_support_fraction=fraction,
        worst=guard.worst,actual_model_state_not_reset_after_initial_placement=True,
        suffix_is_saved_privileged_teacher_goals=True,suffix_is_not_a_learned_closed_loop_policy=True,
        temporary_support_pose_not_working_upper_clearance_certification=True,
        next90_rotation_passed=False,whole_cycle_ready_for_hardware=False,
        devices_opened=0,elapsed_seconds=time.perf_counter()-began)
    save_json(target_folder/'review.json',report);print(json.dumps(report),flush=True)


if __name__=='__main__':main()
