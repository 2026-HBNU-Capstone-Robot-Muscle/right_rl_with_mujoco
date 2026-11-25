"""Offline finite-physics comparison of yaw after the observed handoff.

Image fits are unaccepted hypotheses. No hardware API or device access.
Cube and hand states are initialized once; only finite actuator targets change.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse, hashlib, json, time
from pathlib import Path
from collections import Counter
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS, TICK
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify
from calibrated_quarter_plan_20261028 import LOADED_PATH

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_handoff_yaw_diagnostic'
VISION=ROOT/'right_hand_rotation_20261028/two_axis_connection/actual_transfer_seating_review/review.json'
TRANSFER=ROOT/'right_hand_rotation_20261028/two_axis_connection/operator_pitch_handoff_v3_side_clearance/transfer_retention/front_-10/transfer_path/trace_s1_mu0.3.npz'
WORLD=ROOT/'right_hand_rotation_20261026/saved_marker_registration/model_coordinate_frames.json'

def make_seeds():
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes())
    seed=np.asarray(next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))['settled_cube_qpos'])
    vision=json.loads(VISION.read_bytes());world=json.loads(WORLD.read_bytes());Rwp=np.asarray(world['palm_world_rotation'])
    old=[v for v in vision['observations'] if v['label']=='old_yaw_start' and 'cube_center_palm_m' in v]
    transfer=[v for v in vision['observations'] if v['label'] in ('transfer_end','transfer_hold','transfer_final') and 'cube_center_palm_m' in v]
    assert old and transfer
    old_center=np.median([v['cube_center_palm_m'] for v in old],axis=0)
    new_center=np.median([v['cube_center_palm_m'] for v in transfer],axis=0)
    delta=Rwp@(new_center-old_center)
    old_best=min(old,key=lambda v:v['cube_fit_rms_px']);new_best=min(transfer,key=lambda v:v['cube_fit_rms_px'])
    Rold=np.asarray(old_best['R_cube_world_candidate']);Rnew=np.asarray(new_best['R_cube_world_candidate'])
    Rseed=Rotation.from_quat(np.roll(seed[3:],-1)).as_matrix()
    relative=seed.copy();relative[:3]+=delta
    relative[3:]=np.roll(Rotation.from_matrix(Rseed@Rold.T@Rnew).as_quat(),1)
    raw_orientation=relative.copy();raw_orientation[3:]=np.roll(Rotation.from_matrix(Rnew).as_quat(),1)
    simulated=np.load(TRANSFER)['end_qpos'][-7:]
    seeds={'original_yaw_seed':seed,'retained_model_transfer':simulated,
        'retained_image_relative':relative,'retained_image_raw_orientation':raw_orientation}
    up=-Rwp[:,2]
    for label,base in [('relative',relative),('raw_orientation',raw_orientation)]:
        for height in (4,8,12):
            elevated=base.copy();elevated[:3]+=up*height/1000
            seeds[f'retained_image_{label}_up{height}']=elevated
    for yaw in (-6,0,6):
        for shift in (-3,0,3):
            v=seeds['retained_image_raw_orientation_up12'].copy()
            v[:3]+=Rwp[:,0]*shift/1000
            R=Rotation.from_rotvec(up*np.radians(yaw)).as_matrix()@Rnew
            v[3:]=np.roll(Rotation.from_matrix(R).as_quat(),1)
            seeds[f'image_yaw{yaw:+d}_wrist{shift:+d}']=v
    return seeds,dict(
        hypothesis_delta_palm_mm=((new_center-old_center)*1000).tolist(),hypothesis_delta_world_mm=(delta*1000).tolist(),
        hypothesis_displacement_mm=float(np.linalg.norm(delta)*1000),
        all_image_pose_fits_are_rejected=True,physical_metric_registration_verified=False,
        absolute_camera_translation_not_used=True,common_rigid_mount_translation_cancels_in_relative_delta=True,
        relative_rotation_rebased_on_model_yaw_seed=True,source_sha256=hashlib.sha256(VISION.read_bytes()).hexdigest())

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seeds',nargs='+');parser.add_argument('--path');parser.add_argument('--label',default='original_path');parser.add_argument('--legacy-baseline-settle',action='store_true');parser.add_argument('--image-survey',action='store_true');parser.add_argument('--timestep',type=float,default=.002);parser.add_argument('--period',type=float,default=.075);parser.add_argument('--slow-phases',nargs='+',default=[]);parser.add_argument('--front',type=float,default=-10.);parser.add_argument('--gain',type=float,default=1.);parser.add_argument('--friction',type=float,default=.3);parser.add_argument('--parameter-grid',action='store_true');parser.add_argument('--initial-seed-json');args=parser.parse_args()
    if args.timestep not in (.002,.001,.0005) or args.period not in (.075,.1,.15):raise ValueError('Unsupported diagnostic timing')
    if args.front not in (-15.,-10.,-5.) or args.gain not in (.5,1.,1.5) or args.friction not in (.2,.3,.5):raise ValueError('Unsupported diagnostic physics')
    script_bytes=Path(__file__).read_bytes();script_sha=hashlib.sha256(script_bytes).hexdigest()
    folder=OUT/args.label;folder.mkdir(parents=True,exist_ok=True)
    z=np.load(ROOT/args.path if args.path else LOADED_PATH)
    start=z['start'];goals=z['goals'];phases=[str(v) for v in z['phases']]
    assert np.max(abs(np.diff(np.vstack([start,goals]),axis=0)))<=2
    seeds,seed_meta=make_seeds()
    if args.initial_seed_json:
        seed_path=ROOT/args.initial_seed_json;overrides=json.loads(seed_path.read_bytes())
        for name,values in overrides['seeds'].items():
            value=np.asarray(values,dtype=float)
            if value.shape!=(7,) or not np.isfinite(value).all() or abs(np.linalg.norm(value[3:])-1)>1e-6:raise ValueError('Invalid cube seed hypothesis')
            seeds[name]=value
        seed_meta['additional_seed_source']=args.initial_seed_json;seed_meta['additional_seed_sha256']=hashlib.sha256(seed_path.read_bytes()).hexdigest()
    selected=[k for k in seeds if k.startswith('image_yaw')] if args.image_survey else (args.seeds or ['original_yaw_seed','retained_model_transfer','retained_image_relative','retained_image_raw_orientation'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    (folder/'simulation_runner_snapshot.py.txt').write_bytes(script_bytes)
    m,manifest=build(registry.load('LeapCubeFaceUp',config=cfg),folder/'scene.xml',book_projection_mm=args.front)
    correct_model(m,start);set_cube_physics(m,.055,.1)
    up=np.asarray(json.loads((ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1/seed.json').read_bytes())['up_world'])
    m.opt.timestep=args.timestep;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    m.geom_friction[m.geom('cube').id,0]=.3
    cube=m.geom('cube').id;cb=m.body('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1;low,high=(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK
    gains,bias=m.actuator_gainprm.copy(),m.actuator_biasprm.copy()
    combinations=[(n,g,mu) for n in selected for g in (.5,1.,1.5) for mu in (.2,.3,.5)] if args.parameter_grid else [(n,args.gain,args.friction) for n in selected]
    reports=[];began=time.perf_counter()
    for name,gain,friction in combinations:
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain;m.geom_friction[cube,0]=friction
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seeds[name]
        mujoco.mj_forward(m,d)
        # Reproduce the old baseline's separate two-second warm-up, if asked.
        # Its transient overlap is explicitly recorded and is not newly accepted
        # as an executable loaded entry. Other hypotheses receive no such bypass.
        warmup=None
        if name=='original_yaw_seed' and args.legacy_baseline_settle:
            initial=d.qpos[-7:].copy();maximum=0.
            for _ in range(round(2/args.timestep)):
                mujoco.mj_step(m,d)
                for c in d.contact:
                    if cube in (int(c.geom1),int(c.geom2)):
                        maximum=max(maximum,max(0.,-float(c.dist)*1000))
            warmup=dict(seconds=2.,maximum_unaccepted_initial_overlap_mm=maximum,initial_cube_qpos=initial.tolist(),
                settled_cube_qpos=d.qpos[-7:].tolist(),matches_legacy_baseline_preconditioning=True,
                loaded_entry_approved=False)
        initial_R=d.xmat[cb].reshape(3,3).copy()
        rows=np.vstack([np.repeat(start[None,:],40,axis=0),goals])
        labels=['initial_hold']*40+phases
        sp=su=cp=tracking=excess=gap=longest=0.;external=False;stop=None;worst=None;steps=0
        checkpoints={};trace=[];trace_times=[];contacts=Counter();phase_contacts={};sample=-1;scheduled_time=0.
        for sample,(q,phase) in enumerate(zip(rows,labels)):
            period=.15 if phase in args.slow_phases else args.period
            scheduled_time+=period
            d.ctrl[:]=ticks_to_sim(q);target=round(scheduled_time/args.timestep)
            for _ in range(target-steps):
                before_step=d.qpos.copy()
                mujoco.mj_step(m,d);a,b=inspect(d);filtered=classify(b)
                if filtered and max(filtered.values())>sp:worst=max(filtered,key=filtered.get)
                sp=max(sp,max(filtered.values(),default=0.));su=max(su,max(a.values(),default=0.))
                tracking=max(tracking,float(max(abs(d.qpos[:16]-d.ctrl))))
                excess=max(excess,float(max(0.,max(low-d.qpos[:16]),max(d.qpos[:16]-high))))
                touching=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        cp=max(cp,max(0.,-float(c.dist)*1000));touching|=c.dist<.0004
                        if c.dist<.0004:contacts[m.geom(other).name]+=1
                    elif other in support and c.dist<.0004:external=True
                gap=0. if touching else gap+args.timestep;longest=max(longest,gap)
                if not np.isfinite(d.qpos).all():stop='nonfinite_state'
                elif sp>1e-6:stop='hand_self_contact'
                elif su>.15:stop='hand_support_contact'
                elif cp>1:stop='cube_overlap'
                elif tracking>.18:stop='tracking'
                elif excess>1e-6:stop='native_range'
                elif external:stop='cube_external_contact'
                elif longest>.25:stop='lost_hand_contact'
                if stop:break
            steps=target
            if sample%4==0 or stop:trace.append(d.qpos.copy());trace_times.append(d.time)
            if sample==len(rows)-1 or labels[sample+1]!=phase or stop:
                mujoco.mj_forward(m,d);B=d.xmat[cb].reshape(3,3)
                quat=Rotation.from_matrix(B@initial_R.T).as_quat()
                checkpoints[phase]=dict(sample=sample,cube_qpos=d.qpos[-7:].tolist(),
                    horizontal_twist_deg=float(np.degrees(2*np.arctan2(quat[:3]@up,quat[3]))),
                    red_up_error_deg=float(np.degrees(np.arccos(np.clip(B[:,0]@up,-1,1)))),
                    green_up_error_deg=float(np.degrees(np.arccos(np.clip(-B[:,1]@up,-1,1)))))
                phase_contacts[phase]=dict(contacts);contacts.clear()
            if stop:break
        suffix=f'_g{gain:g}_mu{friction:g}' if args.parameter_grid else ''
        trace_path=folder/(name+suffix+'_trace.npz')
        np.savez_compressed(trace_path,qpos=np.asarray(trace),sample_period_s=4*args.period,end_qpos=d.qpos.copy(),
            failure_contact_qpos=before_step if stop else np.asarray([]),time_s=np.asarray(trace_times))
        held=checkpoints.get('yaw_hold',{})
        report=dict(seed=name,gain_scale=gain,friction=friction,book_front_mm=args.front,stop=stop,motion_health_passed=stop is None,last_phase=labels[sample],samples_completed=sample+1,
            requested_samples=len(rows),max_self_mm=sp,max_support_mm=su,max_cube_overlap_mm=cp,
            max_tracking_rad=tracking,native_range_excess_rad=excess,max_no_hand_contact_seconds=longest,
            external_cube_contact=external,worst_self_pair=worst,phase_checkpoints=checkpoints,phase_cube_contacts=phase_contacts,
            held_horizontal_twist_deg=held.get('horizontal_twist_deg'),trace=trace_path.name,
            initial_cube_qpos=seeds[name].tolist(),final_cube_qpos=d.qpos[-7:].tolist(),legacy_baseline_warmup=warmup)
        reports.append(report)
        record=dict(status='conditional_yaw_from_saved_handoff_hypotheses',cases=reports,seed_metadata=seed_meta,
            model=manifest,path_source=str(ROOT/args.path if args.path else LOADED_PATH),
            path_sha256=hashlib.sha256((ROOT/args.path if args.path else LOADED_PATH).read_bytes()).hexdigest(),
            simulation_script_sha256=script_sha,execution_script_snapshot_captured_at_start=True,
            simulation_timestep_s=args.timestep,command_period_s=args.period,
            slow150ms_phases=args.slow_phases,phase_goal_counts=dict(Counter(phases)),
            nominal_loaded_path_seconds=sum(.15 if p in args.slow_phases else args.period for p in phases),
            finite_actuators=True,hand_pose_prescribed_during_rollout=False,cube_pose_prescribed_during_rollout=False,
            existing_seam_policy_reused=True,hardware_execution_allowed=False,devices_opened=False,
            elapsed_seconds=time.perf_counter()-began)
        (folder/'review.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('phase_checkpoints','phase_cube_contacts')}),flush=True)
    print('SUMMARY',json.dumps(dict(cases=len(reports),motion_health_passes=sum(v['motion_health_passed'] for v in reports),elapsed_s=time.perf_counter()-began)),flush=True)

if __name__=='__main__':main()
