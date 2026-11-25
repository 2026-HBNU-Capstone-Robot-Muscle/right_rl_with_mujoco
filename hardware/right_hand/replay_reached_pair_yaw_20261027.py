"""Free cube response to the new same-grasp yaw candidate; diagnostic only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim, sim_to_hw, TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261027/reached_pair_yaw_study_v2'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate');parser.add_argument('--nominal-only',action='store_true');parser.add_argument('--output-name');args=parser.parse_args();candidate=(ROOT/args.candidate) if args.candidate else OUT/'lift4_dir1.npz';candidate.resolve().relative_to(ROOT);output=candidate.parent/(candidate.stem+'_free_cube_review.json') if args.candidate else OUT/'free_cube_review.json'
    if args.output_name:
        if Path(args.output_name).name!=args.output_name or not args.output_name.endswith('.json'):raise ValueError('Output must be a JSON basename')
        output=candidate.parent/args.output_name
    began=time.perf_counter();source=np.load(OUT/'nominal_grasp_state.npz');data=np.load(candidate);goals=data['goals'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,candidate.parent/(candidate.stem+'_dynamic_scene.xml'));proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;hand={m.body('palm').id}
    low,high=limits();all_goals=np.vstack([data['start'],goals]);max_step=float(np.max(abs(np.diff(all_goals,axis=0))))
    if not len(goals) or not np.isfinite(all_goals).all() or np.any(all_goals<low) or np.any(all_goals>high) or max_step>2:raise ValueError('Candidate fails finite native range / 2 tick validation')
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();reports=[]
    for scale in ((1.,) if args.nominal_only else (1.,.5,1.5)):
      for mu in ((.3,) if args.nominal_only else (.3,.2,.5)):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;m.geom_friction[cube,0]=mu;d=mujoco.MjData(m);d.qpos[:]=source['qpos'];d.qvel[:]=source['qvel'];d.ctrl[:]=source['ctrl']
        for _ in range(1000):mujoco.mj_step(m,d)
        C=d.qpos[-7:-4].copy();B=d.xmat[m.body('cube').id].reshape(3,3).copy();trace=[];maxself=maxsup=maxcp=maxtrack=maxyaw=maxrange=maxshift=nohand=longest_nohand=0.;external=False;stop=None;steps=0
        for i,q in enumerate(np.vstack([goals,np.repeat(goals[-1][None,:],40,axis=0)])):
            goal=ticks_to_sim(q);d.ctrl[:]=goal;n=round((i+1)*.075/.002)-steps;steps+=n
            for _ in range(n):
                mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.));maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-goal))));raw=sim_to_hw(d.qpos[:16])/TICK;maxrange=max(maxrange,float(np.max(np.maximum(low-raw,raw-high))));maxshift=max(maxshift,float(np.linalg.norm(d.qpos[-7:-4]-C)*1000));inhand=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube in (a,b):
                        other=b if a==cube else a;dep=max(0.,-float(c.dist)*1000)
                        if int(m.geom_bodyid[other]) in hand:
                            maxcp=max(maxcp,dep)
                            if c.dist<.0004:inhand=True
                        elif other in support and c.dist<.0004:external=True
                nohand=0. if inhand else nohand+.002;longest_nohand=max(longest_nohand,nohand)
                if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1 or external or maxshift>80 or longest_nohand>.25:stop='contact/tracking/range/retention violation';break
            R=d.xmat[m.body('cube').id].reshape(3,3);delta=Rotation.from_matrix(R@B.T).as_quat();yaw=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])));maxyaw=max(maxyaw,yaw);tilt=float(np.degrees(np.arccos(np.clip((R@np.array([0.,-1,0]))@(B@np.array([0.,-1,0])),-1,1))))
            if i%20==0 or stop or i==len(goals)+39:trace.append(dict(sample=i+1,yaw_deg=yaw,tilt_change_deg=tilt,height_change_mm=float((d.qpos[-7:-4]-C)@up*1000)))
            if stop:break
        row=dict(gain_scale=scale,friction=mu,stop=stop,samples=i+1,complete=stop is None,max_yaw_deg=maxyaw,final_yaw_deg=yaw,final_tilt_change_deg=tilt,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,max_actual_native_excess_ticks=maxrange,max_cube_translation_mm=maxshift,longest_no_hand_contact_seconds=longest_nohand,external_cube_contact=external,trace=trace);reports.append(row);print(json.dumps({k:v for k,v in row.items() if k!='trace'}),flush=True)
    output.write_text(json.dumps(dict(status='free_cube_yaw_diagnostic',cases=reports,candidate=candidate.relative_to(ROOT).as_posix(),maximum_command_step_ticks=max_step,finite_native_goals_verified=True,actual_native_excess_tolerance_ticks=1,geometric_target_reached_deg=float(max(data['desired_angles'])) if 'desired_angles' in data else 66,physical_yaw_verified=False,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
