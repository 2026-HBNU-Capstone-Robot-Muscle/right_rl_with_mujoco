"""Run the packaged NumPy stages in free native physics, never on hardware."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha,motor,ticks_to_sim
from staged_shadow_runtime import StagedShadowController,AdmissibleShadowController
from shadow_runtime import SIM_IDS,TICK
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg,face_up_error_deg
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--case',choices=('nominal','y_plus05','friction035'),default='nominal')
    p.add_argument('--integer-encoders',action='store_true')
    p.add_argument('--hold-extra-steps',type=int,default=0)
    p.add_argument('--correction-actor-tag',help='Research actor override; does not certify or replace the packaged actor')
    p.add_argument('--admissible-correction',action='store_true',help='Research comparison selecting the nearest native-range goal before unchanged guards')
    p.add_argument('--tag-suffix',help='Preserve a separate confirmation trial')
    p.add_argument('--compare-goals-to',help='Relative prior research_trace.npz to compare without driving its recorded goals')
    p.add_argument('--model-kind',choices=('active','active_ccd50'),default='active')
    p.add_argument('--rotation-repair-export',help='Relative SAC export; gated to reference phases3000..4299 in a CCD50 model comparison')
    a=p.parse_args()
    if a.rotation_repair_export and (a.model_kind!='active_ccd50' or not a.integer_encoders):
        raise ValueError('Repair requires its CCD50 and integer-encoder research contract')
    label=a.case+('_integer_encoders' if a.integer_encoders else '_fractional_encoders')
    if a.correction_actor_tag:label+='_'+a.correction_actor_tag
    if a.admissible_correction:label+='_admissible_correction'
    if a.hold_extra_steps:label+=f'_extra_hold{a.hold_extra_steps}'
    if a.tag_suffix:label+='_'+a.tag_suffix
    if a.model_kind!='active':label+='_'+a.model_kind
    folder=OUT/'next_cycle'/('staged_model_'+label);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed trial')
    (folder/'source_evaluate_staged_shadow_model.py').write_bytes(Path(__file__).read_bytes())
    bundle=OUT/('next_cycle/staged_shadow_bundle_v2_ccd50' if a.model_kind=='active_ccd50' else 'next_cycle/staged_shadow_bundle_v2')
    c=(AdmissibleShadowController if a.admissible_correction else StagedShadowController)(bundle)
    repair_path=None;repair_weights=None
    original_rotation_weights=(c.mean,c.std,c.layers)
    if a.rotation_repair_export:
        repair_folder=OUT/a.rotation_repair_export
        repair_manifest=json.loads((repair_folder/'manifest.json').read_text())
        repair_path=repair_folder/'actor.npz'
        if not repair_manifest['parity_passed'] or sha(repair_path)!=repair_manifest['actor_sha256'] or repair_manifest['current_model_sha256']!=c.meta['model_sha256'] or repair_manifest['residual_rad']!=c.meta['residual_rad']:
            raise ValueError('Repair export provenance or inference contract differs')
        with np.load(repair_path,allow_pickle=False) as data:
            repair_weights=(data['mean'],data['std'],[(data[f'w{i}'],data[f'b{i}']) for i in range(3)])
    actor_path=bundle/'correction_actor.npz'
    if a.correction_actor_tag:
        actor_path=OUT/'next_cycle'/a.correction_actor_tag/'actor.npz'
        with np.load(actor_path,allow_pickle=False) as actor:
            c.cmean=actor['mean'];c.cstd=actor['std'];c.clayers=[(actor[f'w{i}'],actor[f'b{i}']) for i in range(3)]
    m,z,_,_,provenance=assets(a.model_kind);guard=Guard(m);cube=m.geom('cube').id
    if a.case=='friction035':
        for gid in range(m.ngeom):
            if int(m.geom_bodyid[gid]) in guard.hand or gid==cube:m.geom_friction[gid,0]=.35
    d=restore(m,z)
    if a.case=='y_plus05':d.qpos[-7:-4]+=[0.,.0005,0.];mujoco.mj_forward(m,d)
    geometry=precompute(m,guard.hand);certificate=SweepCertificate(m,guard.hand,.3)
    target=upright_yaw_target(z['initial_cube_rotation'],z['up'],90)
    steps=6556+max(0,a.hold_extra_steps);seq=0;goals=[];stages=[];trace=[];stop=None
    hold=dict(samples=0,lower=0,bad=0,gap=float('inf'),orientation_drift_deg=0.,center_drift_mm=0.)
    hold_B=hold_C=None;began=time.perf_counter()

    def status():
        B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube]
        return dict(error_deg=orientation_error_deg(B,target),tilt_deg=face_up_error_deg(B,5,z['up']),
                    center_error_mm=float(np.linalg.norm(C-c.station)*1000),
                    upper_gap_mm=min(gap for _,gap in gaps(m,d,C,B,geometry,True)))

    def sensor_sample():
        measured=np.empty(16);measured[SIM_IDS-1]=d.qpos[:16]/TICK+2048
        if a.integer_encoders:measured=np.rint(measured)
        q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[cube])
        s=dict(sequence=seq,sample_time=seq*.075,pose_time=seq*.075,
               frame_id='leap_sim_world',palm_point='policy_palm_position',
               measured_ticks=measured,commanded_ticks=motor(d.ctrl),
               palm_position=np.array([.11,0.,.03]),cube_position=d.geom_xpos[cube].copy(),cube_quaternion=q)
        if c.phase>=c.prefix_steps:
            # Screening is calculated with full model geometry. It is not a
            # global distance certificate. Strict certification is separate.
            s['upper_sweep_acquisition']=dict(model_sha256=c.meta['model_sha256'],sequence=seq,
                    screen_mm=status()['upper_gap_mm'],method='slab_clipped_radial_screen')
        return s

    # The first three model samples are identical. No claim of physically fresh
    # camera sampling or advancing the actual system while warming up is made.
    for _ in range(2):
        assert c.propose(sensor_sample())['state']=='warming';seq+=1
    for i in range(steps):
        c.mean,c.std,c.layers=(repair_weights if repair_weights is not None and 3000<=c.phase<4300 else original_rotation_weights)
        try:proposal=c.propose(sensor_sample())
        except ValueError as error:stop=['shadow_contract_rejection',str(error)];break
        goal=np.asarray(proposal['goal_ticks'])
        if np.max(abs(goal-motor(d.ctrl)))>2:raise AssertionError('Two tick contract')
        d.ctrl[:]=ticks_to_sim(goal)
        c.commit_shadow(seq,goal);seq+=1;goals.append(goal);stages.append(c.mode)
        for _ in range(75):
            mujoco.mj_step(m,d);stop=guard.check(d)
            if np.linalg.norm(d.qpos[-7:-4]-z['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
            if i>=steps-54:
                B=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];forces={}
                for ci,contact in enumerate(d.contact):
                    u,v=int(contact.geom1),int(contact.geom2)
                    if cube not in (u,v):continue
                    name=m.geom(v if u==cube else u).name
                    if name in ('if_tip','th_tip') and -(contact.pos-C)@B[:,1]*1000<55/6:
                        f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);forces[name]=forces.get(name,0.)+float(f[0])
                hold['samples']+=1;hold['lower']+=all(forces.get(name,0.)>.2 for name in ('if_tip','th_tip'))
                gap,bad=certificate.query(d,C,B);hold['gap']=min(hold['gap'],gap);hold['bad']+=bool(bad)
                if hold_B is None:hold_B=B.copy();hold_C=C.copy()
                hold['orientation_drift_deg']=max(hold['orientation_drift_deg'],orientation_error_deg(B,hold_B))
                hold['center_drift_mm']=max(hold['center_drift_mm'],float(np.linalg.norm(C-hold_C)*1000))
            if stop:break
        if (i+1)%1000==0:print(json.dumps(dict(case=label,steps=i+1,stage=c.mode,stop=stop)),flush=True)
        if i%100==0 or i>=6300 or stop:trace.append(dict(step=i+1,stage=c.mode,**status()))
        if stop:break
    final=status();reasons=[]
    if stop:reasons.append('native_or_shadow_guard')
    if len(goals)!=steps:reasons.append('incomplete_run')
    if c.mode!='hold':reasons.append('fine_acquisition_not_reached')
    if final['error_deg']>=5:reasons.append('quarter_orientation')
    if final['tilt_deg']>=5:reasons.append('up_tilt')
    if final['center_error_mm']>=2:reasons.append('common_station')
    if hold['samples']!=4050:reasons.append('hold_incomplete')
    if hold['lower']/max(1,hold['samples'])<=.8:reasons.append('lower_contacts')
    if hold['bad'] or hold['gap']<=.3:reasons.append('upper_space_certificate')
    if hold['orientation_drift_deg']>=3 or hold['center_drift_mm']>=2:reasons.append('hold_drift')
    if not np.isfinite(hold['gap']):hold['gap']=None
    switches=[dict(step=i+1,stage=s) for i,s in enumerate(stages) if i==0 or s!=stages[i-1]]
    np.savez_compressed(folder/'research_trace.npz',goals=np.asarray(goals),stages=np.asarray(stages))
    np.savez_compressed(folder/'end_state.npz',**{key:getattr(d,key) for key in BOUND},time=d.time,
                        up=z['up'],initial_cube_rotation=z['initial_cube_rotation'],initial_cube_center=z['initial_cube_center'])
    result=dict(status='packaged_controller_free_model_trial',case=a.case,integer_encoder_observations=a.integer_encoders,
                model_kind=a.model_kind,
                case_used_for_correction_teacher_data=a.case=='nominal',
                requested_steps=steps,steps=len(goals),first_rotation_and_hold_passed=not reasons,reasons=reasons,stop=stop,
                final=final,final_hold=hold,worst=guard.worst,stage_switches=switches,trace=trace,
                bundle_metadata_sha256=sha(bundle/'bundle.json'),model_provenance=provenance,
                correction_actor_path=str(actor_path.relative_to(OUT)),correction_actor_sha256=sha(actor_path),
                correction_actor_research_override=a.correction_actor_tag,
                 rotation_repair_actor_path=str(repair_path.relative_to(OUT)) if repair_path else None,
                 rotation_repair_actor_sha256=sha(repair_path) if repair_path else None,
                 rotation_repair_phase_gate=[3000,4300] if repair_path else None,
                native_range_admissible_correction_selection=a.admissible_correction,
                admissible_correction_events=getattr(c,'projection_events',[]),
                certificate=certificate.metadata(),no_state_reset_after_initial_placement=True,
                exact_model_object_pose_and_geometry_assumed=True,physical_sensor_accuracy_validated=False,
                upper_space_provider_is_privileged_model_geometry=True,
                acquisition_screen_is_global_clearance_certificate=False,
                history_warmup_uses_repeated_initial_model_samples=True,
                next_rotation_passed=False,complete_cycle_passed=False,devices_opened=0,hardware_execution_allowed=False,
                elapsed_seconds=time.perf_counter()-began)
    if a.compare_goals_to:
        old_path=OUT/a.compare_goals_to;expected=np.load(old_path,allow_pickle=False)['goals']
        same_length=len(expected)==len(goals)
        result['independently_generated_goals_comparison']=dict(source_path=a.compare_goals_to,source_sha256=sha(old_path),
                same_length=same_length,mismatched_goal_rows=int(np.count_nonzero(np.any(expected!=np.asarray(goals),axis=1))) if same_length else None,
                prior_recorded_goals_were_not_used_to_generate_commands=True)
    save_json(folder/'review.json',result)
    print(json.dumps({key:value for key,value in result.items() if key not in ('trace','model_provenance','admissible_correction_events')}),flush=True)


if __name__=='__main__':main()
