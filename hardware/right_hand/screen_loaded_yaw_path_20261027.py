"""Whole loaded grasp/yaw/open physics review; no robot or camera access."""
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
from screen_loaded_repeat_20261026 import cached_contacts
from book_empty_preparation_plan import FRONTS
ROOT=Path(__file__).resolve().parent

def interpolate(a,b):
    count=max(1,int(np.ceil(max(abs(b-a))/2)))
    return np.rint([a+(b-a)*i/count for i in range(1,count+1)]).astype(int)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',required=True);parser.add_argument('--output-folder',required=True);parser.add_argument('--nominal-only',action='store_true');parser.add_argument('--front',type=float);args=parser.parse_args()
    source=ROOT/args.candidate;source.resolve().relative_to(ROOT);out=ROOT/args.output_folder;out.resolve().relative_to(ROOT);out.mkdir(parents=True,exist_ok=True);began=time.perf_counter();data=np.load(source)
    opened=np.load(OPEN_PATH)['open_goal'];z=np.load(GRASP_PATH);grasp=z['goals'][z['phases']=='empty_new_grasp'].copy();grasp[:,7]=np.maximum(grasp[:,7],2048)
    assert np.array_equal(data['start'],grasp[-1]);yaw=data['goals'].astype(int);release=interpolate(yaw[-1],opened)
    pieces=[('grasp',grasp),('grasp_hold',np.repeat(grasp[-1][None,:],40,axis=0)),('yaw',yaw),('yaw_hold',np.repeat(yaw[-1][None,:],40,axis=0)),('release',release),('remove_hold',np.repeat(opened[None,:],40,axis=0))]
    goals=np.vstack([q for _,q in pieces]);phases=np.concatenate([np.repeat(p,len(q)) for p,q in pieces]);lo,hi=limits();values=np.vstack([opened,goals]);assert np.isfinite(values).all() and np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2
    np.savez_compressed(out/'loaded_path.npz',start=opened,goals=goals,phases=phases,open_goal=opened,grasp_goal=grasp[-1],yaw_goal=yaw[-1])
    prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);simids=np.array(SIM_TO_MOTOR_IDS)-1;nativelo=(lo[simids]-2048)*TICK;nativehi=(hi[simids]-2048)*TICK
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);reports=[]
    for front in ((args.front,) if args.front is not None else ((-10.,) if args.nominal_only else FRONTS)):
        m,_=build(env,out/f'scene_{front:g}.xml',book_projection_mm=front);correct_model(m,np.array(prior['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in ((1.,) if args.nominal_only else (1.5,1.,.5)):
          for friction in ((.3,) if args.nominal_only else (.2,.3,.5)):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;m.geom_friction[cube,0]=friction
            seed=next(r for r in prior['loaded_physics'] if (r['book_front_mm'],r['gain_scale'],r['friction'])==(front,scale,friction));d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0
            for _ in range(1000):mujoco.mj_step(m,d)
            B=d.xmat[m.body('cube').id].reshape(3,3).copy();C=d.qpos[-7:-4].copy();yawB=None;maxself=maxsup=maxcp=maxtrack=maxrange=nohand=longest=0.;external=False;stopped=None;steps=0;worst={};turn_angle=None;turn_tilt=None
            def angle(R,base):
                delta=Rotation.from_matrix(R@base.T).as_quat();return float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
            for sample,(phase,q) in enumerate(zip(phases,goals),1):
                d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/.002)
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
                    nohand=0. if inhand else nohand+.002;longest=max(longest,nohand)
                    if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25:stopped='contact/range/tracking/retention violation';break
                steps=target
                if phase=='grasp_hold':yawB=d.xmat[m.body('cube').id].reshape(3,3).copy()
                if phase=='yaw_hold' and yawB is not None:
                    R=d.xmat[m.body('cube').id].reshape(3,3);turn_angle=angle(R,yawB);turn_tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(yawB@np.array([0.,-1,0])),-1,1))))
                if stopped:break
            R=d.xmat[m.body('cube').id].reshape(3,3);shift=float(np.linalg.norm(d.qpos[-7:-4]-C)*1000);passed=bool(stopped is None and inhand and shift<80)
            row=dict(book_front_mm=front,gain_scale=scale,friction=friction,passed=passed,stop=stopped,last_phase=str(phase),last_sample=sample,yaw_before_release_deg=turn_angle,yaw_tilt_change_deg=turn_tilt,final_yaw_from_placement_deg=angle(R,B),max_self_mm=maxself,worst_self_pairs_mm=worst,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,unapproved_range_excess_rad=max(0,maxrange),external_cube_contact=external,final_cube_supported_by_hand=bool(inhand),center_translation_mm=shift,longest_no_hand_contact_seconds=longest);reports.append(row);print(json.dumps(row),flush=True)
            result=dict(status='full_loaded_yaw_release_model_diagnostic',candidate=source.relative_to(ROOT).as_posix(),candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),loaded_path_sha256=hashlib.sha256((out/'loaded_path.npz').read_bytes()).hexdigest(),goals=len(goals),nominal_loaded_seconds=len(goals)*.075,cases=reports,all_passed=all(r['passed'] for r in reports),finite_stiffness_contact_forces_preserved=True,physical_yaw_verified=False,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
            (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('ALL',len(reports),result['all_passed'],flush=True)
if __name__=='__main__':main()
