"""Continuous forward/inverse loaded review; no robot or camera access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from new_pair_grasp_plan_20261027 import OPEN_PATH,GRASP_PATH
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from cube_contact_material_policy_20261029 import apply_material, cube_contact_friction
from screen_loaded_repeat_20261026 import cached_contacts
from book_empty_preparation_plan import FRONTS
ROOT=Path(__file__).resolve().parent

def interpolate(a,b):
    count=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/count for i in range(1,count+1)]).astype(int)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',required=True);parser.add_argument('--output-folder',required=True);parser.add_argument('--nominal-only',action='store_true');parser.add_argument('--front',type=float);parser.add_argument('--dt',type=float,default=.001,choices=(.002,.001,.0005));parser.add_argument('--gain',type=float,choices=(.5,1.,1.5));parser.add_argument('--friction',type=float,choices=(.2,.3,.5));parser.add_argument('--cube-seed-json');parser.add_argument('--offset-mm',type=float,nargs=3,default=(0.,0.,0.));parser.add_argument('--seat-up',action='store_true');args=parser.parse_args()
    source=ROOT/args.candidate;source.resolve().relative_to(ROOT);out=ROOT/args.output_folder;out.resolve().relative_to(ROOT);out.mkdir(parents=True,exist_ok=True);began=time.perf_counter();data=np.load(source)
    opened=data['start'].astype(int);assert opened.shape==(16,)
    goals=data['goals'].astype(int)
    phase_map={'stage_5_loaded_grasp':'grasp','stage_6_grasp_hold':'grasp_hold','stage_7_horizontal_yaw':'yaw','stage_8_horizontal_hold':'yaw_hold','stage_9_loaded_open':'release','stage_10_remove_hold':'remove_hold'}
    phases=np.asarray([phase_map[str(p)] for p in data['phases']]);grasp=goals[phases=='grasp'];yaw=goals[phases=='yaw']
    lo,hi=limits();values=np.vstack([opened,goals]);assert np.isfinite(values).all() and np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2
    np.savez_compressed(out/'loaded_path.npz',start=opened,goals=goals,phases=phases,open_goal=opened,grasp_goal=grasp[-1],yaw_goal=(yaw[-1] if len(yaw) else grasp[-1]))
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);simids=np.array(SIM_TO_MOTOR_IDS)-1;nativelo=(lo[simids]-2048)*TICK;nativehi=(hi[simids]-2048)*TICK
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);reports=[]
    for front in ((args.front,) if args.front is not None else ((-10.,) if args.nominal_only else FRONTS)):
        m,_=build(env,out/f'scene_{front:g}.xml',book_projection_mm=front);correct_model(m,np.array(prior['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=args.dt;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in ((args.gain,) if args.gain is not None else ((1.,) if args.nominal_only else (1.5,1.,.5))):
          for friction in ((args.friction,) if args.friction is not None else ((.3,) if args.nominal_only else (.2,.3,.5))):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;material=apply_material(m,friction,hand,cube)
            seed=next(r for r in prior['loaded_physics'] if (r['book_front_mm'],r['gain_scale'],r['friction'])==(front,scale,friction));d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0
            if args.cube_seed_json:
                seed_path=ROOT/args.cube_seed_json;seed_path.resolve().relative_to(ROOT)
                d.qpos[-7:]=json.loads(seed_path.read_bytes())['cube_qpos']
            d.qpos[-7:-4]+=np.asarray(args.offset_mm)/1000.
            seating_shift=0.
            if args.seat_up:
                base_center=d.qpos[-7:-4].copy();upper=.04;lower=None;previous=upper
                def hand_depth(height):
                    d.qpos[-7:-4]=base_center+up*height;mujoco.mj_forward(m,d);depth=0.
                    for contact in d.contact:
                        a,b=int(contact.geom1),int(contact.geom2)
                        if cube in (a,b) and int(m.geom_bodyid[b if a==cube else a]) in hand:depth=max(depth,max(0.,-float(contact.dist)*1000))
                    return depth
                if hand_depth(upper)>1e-5:raise RuntimeError('Upper seating endpoint already intersects hand; reject vertical seating')
                for height in np.arange(upper,-.025,-.0005):
                    if hand_depth(float(height))>1e-5:lower=float(height);upper=float(previous);break
                    previous=float(height)
                if lower is None:raise RuntimeError('No hand support found along bounded vertical seating line')
                for _ in range(24):
                    mid=(lower+upper)*.5
                    if hand_depth(mid)>1e-5:lower=mid
                    else:upper=mid
                seating_shift=upper+.0001;d.qpos[-7:-4]=base_center+up*seating_shift
            placement_probe=dict(max_self_mm=0.,max_support_mm=0.,max_cube_mm=0.,max_tracking_rad=0.,external_cube_contact=False)
            def probe_placement():
                su,sp=inspect(d);placement_probe['max_self_mm']=max(placement_probe['max_self_mm'],max(sp.values(),default=0.));placement_probe['max_support_mm']=max(placement_probe['max_support_mm'],max(su.values(),default=0.))
                placement_probe['max_tracking_rad']=max(placement_probe['max_tracking_rad'],float(max(abs(d.qpos[:16]-d.ctrl))))
                for contact in d.contact:
                    a,b=int(contact.geom1),int(contact.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:placement_probe['max_cube_mm']=max(placement_probe['max_cube_mm'],max(0.,-float(contact.dist)*1000))
                    elif other in support and contact.dist<.0004:placement_probe['external_cube_contact']=True
            mujoco.mj_forward(m,d);probe_placement()
            for _ in range(round(2./args.dt)):mujoco.mj_step(m,d);probe_placement()
            placement_ok=bool(placement_probe['max_self_mm']<=1e-6 and placement_probe['max_support_mm']<=.15 and placement_probe['max_cube_mm']<=1 and placement_probe['max_tracking_rad']<=.18 and not placement_probe['external_cube_contact'])
            material['effective_contacts_after_settle']=cube_contact_friction(m,d,hand,cube);B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.qpos[-7:-4].copy();yawB=None;maxself=maxsup=maxcp=maxtrack=maxrange=nohand=longest=0.;external=False;stopped=None;steps=0;worst={};turn_angle=None;turn_tilt=None;whole_turn=None;forward_whole_turn=None;inverse_delta=None;forward_end=int(data['forward_end_sample']);grasp_R=None;grasp_C=None;grasp_steps=0;grasp_eval_steps=0;grasp_drift=grasp_shift=0.;grasp_counts={'mf_tip':0,'th_tip':0}
            def angle(R,base):
                delta=Rotation.from_matrix(R@base.T).as_quat();return float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
            for sample,(phase,q) in enumerate(zip(phases,goals),1):
                d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/args.dt)
                for _ in range(target-steps):
                    mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.));maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-d.ctrl))));maxrange=max(maxrange,float(max(np.maximum(nativelo-d.qpos[:16],d.qpos[:16]-nativehi))));inhand=False
                    for pair,depth in sp.items():worst[pair]=max(worst.get(pair,0.),depth)
                    for c in d.contact:
                        a,b=int(c.geom1),int(c.geom2)
                        if cube in (a,b):
                            other=b if a==cube else a
                            if int(m.geom_bodyid[other]) in hand:
                                maxcp=max(maxcp,max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                            elif other in support and c.dist<.0004:external=True
                    nohand=0. if inhand else nohand+args.dt;longest=max(longest,nohand)
                    if phase=='grasp_hold':
                        grasp_steps+=1
                        if grasp_steps>=round(1./args.dt):
                            if grasp_R is None:grasp_R=d.xmat[m.body('cube').id].reshape(3,3).copy();grasp_C=d.qpos[-7:-4].copy()
                            grasp_eval_steps+=1;grasp_drift=max(grasp_drift,abs(angle(d.xmat[m.body('cube').id].reshape(3,3),grasp_R)));grasp_shift=max(grasp_shift,float(np.linalg.norm(d.qpos[-7:-4]-grasp_C)*1000));normal={}
                            for ci,con in enumerate(d.contact):
                                aa,bb=int(con.geom1),int(con.geom2)
                                if cube not in (aa,bb):continue
                                other=bb if aa==cube else aa;name=m.geom(other).name
                                if name in grasp_counts:
                                    force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0])
                            for name in grasp_counts:grasp_counts[name]+=normal.get(name,0.)>.2
                    if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25:stopped='contact/range/tracking/retention violation';break
                steps=target
                if phase=='grasp_hold':yawB=d.xmat[m.body('cube').id].reshape(3,3).copy()
                if phase=='yaw_hold' and yawB is not None:
                    R=d.xmat[m.body('cube').id].reshape(3,3);whole_turn=angle(R,B);turn_angle=angle(R,yawB);turn_tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(yawB@np.array([0.,-1,0])),-1,1))))
                    if sample<=forward_end:forward_whole_turn=whole_turn
                    elif forward_whole_turn is not None:inverse_delta=whole_turn-forward_whole_turn
                if stopped:break
            R=d.xmat[m.body('cube').id].reshape(3,3);shift=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000);passed=bool(stopped is None and inhand and shift<80 and placement_ok)
            if stopped is None and not placement_ok:stopped='initial_placement_contact_or_tracking'
            grasp_task=dict(evaluated_steps=grasp_eval_steps,max_yaw_drift_deg=grasp_drift,max_center_shift_mm=grasp_shift,opposing_contact_fraction={k:v/grasp_eval_steps if grasp_eval_steps else 0 for k,v in grasp_counts.items()},passed=bool(grasp_eval_steps and grasp_drift<3 and grasp_shift<2 and all(v/grasp_eval_steps>.8 for v in grasp_counts.values())))
            row=dict(grasp_hold_observation=grasp_task,vertical_seating_line_upper_verified_free=bool(args.seat_up),forward_from_placement_deg=forward_whole_turn,inverse_from_forward_hold_deg=inverse_delta,material_audit=material,seating_height_shift_mm=seating_shift*1000,initial_placement_probe=placement_probe,initial_placement_passed=placement_ok,cube_seed_json=args.cube_seed_json,initial_cube_after_settle=C.tolist(),seed_offset_mm=list(args.offset_mm),simulation_timestep_s=args.dt,book_front_mm=front,gain_scale=scale,friction=friction,passed=passed,stop=stopped,last_phase=str(phase),last_sample=sample,placed_to_turn_deg=whole_turn,yaw_before_release_deg=turn_angle,yaw_tilt_change_deg=turn_tilt,final_yaw_from_placement_deg=angle(R,B),max_self_mm=maxself,worst_self_pairs_mm=worst,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,unapproved_range_excess_rad=max(0,maxrange),external_cube_contact=external,final_cube_supported_by_hand=bool(inhand),center_translation_mm=shift,longest_no_hand_contact_seconds=longest);reports.append(row);print(json.dumps(row),flush=True)
            result=dict(material_policy='both_contacting_surfaces_v1_not_physical_identification',parent_model_sha256='28241b29264f5ae644b22588cb5afd276cea0a0104b3df15153e8adf85b8e5f2',simulation_timestep_s=args.dt,settling_seconds=2.,initial_placement_also_checked=True,physics_acceptance_bounds_unchanged=True,cube_initial_local_green_axis_turn_deg=0,cube_seed_json=args.cube_seed_json,cube_seed_sha256=hashlib.sha256((ROOT/args.cube_seed_json).read_bytes()).hexdigest() if args.cube_seed_json else None,loaded_route='phase-labelled_joint_cycle',model_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),status='natural_cradle_free_grasp_hold_release_diagnostic',candidate=source.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),loaded_path_sha256=hashlib.sha256((out/'loaded_path.npz').read_bytes()).hexdigest(),goals=len(goals),nominal_loaded_seconds=len(goals)*.075,cases=reports,all_passed=all(r['passed'] for r in reports),finite_stiffness_contact_forces_preserved=True,physical_yaw_verified=False,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
            (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('ALL',len(reports),result['all_passed'],flush=True)
if __name__=='__main__':main()
