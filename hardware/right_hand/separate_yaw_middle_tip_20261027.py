"""Offline finger-only clearance before the last yaw segment, no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,sim_to_hw,TICK
from loaded_quarter_reentry_20261025 import limits
from loaded_quarter_model_policy_20261025 import correct_model
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'right_hand_rotation_20261027/pair_yaw_finish_contact'
PREFIX=ROOT/'right_hand_rotation_20261027/pair_free_yaw_optimization_v3/candidate.npz'
OUT=ROOT/'right_hand_rotation_20261027/pair_yaw_middle_separation'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--finish-yaw',action='store_true');args=parser.parse_args();output=OUT.with_name('pair_yaw_finish_root_offsets') if args.finish_yaw else OUT
    began=time.perf_counter();output.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';m,_=build(registry.load('LeapCubeFaceUp',config=cfg),output/'scene.xml')
    proof=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_bytes());correct_model(m,np.array(proof['start_ticks']));set_cube_physics(m,.055,.1);m.opt.timestep=.002
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;m.geom_friction[cube,0]=.3;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cached=np.load(SOURCE/'prefix_end_state.npz');prefix=np.load(OUT/'candidate.npz' if args.finish_yaw else PREFIX);q=prefix['goals'][-1];B=cached['base_rotation'];C=cached['base_center'];initial_state=np.load(OUT/'end_state.npz')['integration'] if args.finish_yaw else cached['integration'];kind=mujoco.mjtState.mjSTATE_INTEGRATION;d=mujoco.MjData(m);size=mujoco.mj_stateSize(m,kind);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);low,high=limits();pair=[m.geom(n).id for n in ('mf_tip','palm_collision_6')];rows=[];best=None
    finish_goal=None
    if args.finish_yaw:
        finish_review=json.loads((ROOT/'right_hand_rotation_20261027/pair_free_yaw_optimization_v6/review.json').read_bytes());last=finish_review['steps'][-1];finish_goal=np.array((last['rejected'][0] if 'rejected' in last else last)['goal'])
    for motor in ((12,) if args.finish_yaw else (9,10,11,12)):
      for delta in ((-4,-8,-16,-24,-32) if args.finish_yaw else (-32,-16,-8,8,16,32)):
        goal=finish_goal.copy() if args.finish_yaw else q.copy();goal[motor-1]+=delta
        if np.any(goal<low) or np.any(goal>high):continue
        count=max(60 if args.finish_yaw else 1,int(np.ceil(max(abs(goal-q))/2)));path=np.rint([q+(goal-q)*k/count for k in range(1,count+1)]).astype(int);path=np.vstack([path,np.repeat(goal[None,:],40,axis=0)])
        mujoco.mj_setState(m,d,initial_state,kind);mujoco.mj_forward(m,d);maxself=maxsup=maxcube=maxtrack=maxrange=0.;mingap=1e9;external=False;steps=0;worst={}
        for i,command in enumerate(path,1):
            d.ctrl[:]=ticks_to_sim(command);target=round(i*.075/.002)
            for _ in range(target-steps):
                mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.));maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-d.ctrl))));raw=sim_to_hw(d.qpos[:16])/TICK;maxrange=max(maxrange,float(max(np.maximum(low-raw,raw-high))));mingap=min(mingap,float(mujoco.mj_geomDistance(m,d,*pair,1.,None)*1000))
                for name,depth in sp.items():worst[name]=max(worst.get(name,0.),depth)
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube in (a,b):
                        other=b if a==cube else a
                        if int(m.geom_bodyid[other]) in hand:maxcube=max(maxcube,max(0.,-float(c.dist)*1000))
                        elif other in support and c.dist<.0004:external=True
            steps=target
        R=d.xmat[m.body('cube').id].reshape(3,3);dq=Rotation.from_matrix(R@B.T).as_quat();yaw=float(np.degrees(2*np.arctan2(dq[:3]@up,dq[3])));gap=float(mujoco.mj_geomDistance(m,d,*pair,1.,None)*1000)
        passed=bool(maxself<=1e-6 and maxsup<=.15 and maxcube<=1 and maxtrack<=.18 and maxrange<=1e-6 and not external and gap>=.2 and yaw>=(85 if args.finish_yaw else 82) and (yaw<=95 if args.finish_yaw else True))
        row=dict(motor_id=motor,delta_ticks=delta,yaw_deg=yaw,final_gap_mm=gap,minimum_gap_mm=mingap,max_self_mm=maxself,self_pairs_mm=worst,max_cube_mm=maxcube,max_tracking_rad=maxtrack,actual_native_excess_ticks=maxrange,passed=passed);rows.append(row)
        print(json.dumps(row),flush=True)
        better=best is None or (abs(yaw-90)<abs(best[0]['yaw_deg']-90) if args.finish_yaw else gap>best[0]['final_gap_mm'])
        if passed and better:
            state=np.empty(size);mujoco.mj_getState(m,d,state,kind);best=row,path.copy(),state
    result=dict(status='middle_tip_pre_yaw_clearance_diagnostic',cases=rows,candidate_found=best is not None,hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-began)
    if best:
        row,path,state=best;goals=np.vstack([prefix['goals'],path]);np.savez_compressed(output/'candidate.npz',start=prefix['start'],goals=goals,desired_angles=np.r_[prefix['desired_angles'],np.repeat(row['yaw_deg'],len(path))],bias_ticks=prefix['bias_ticks']);np.savez_compressed(output/'end_state.npz',integration=state);result['selected']=row
    (output/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print('SELECTED',json.dumps(result.get('selected')),flush=True)
if __name__=='__main__':main()
