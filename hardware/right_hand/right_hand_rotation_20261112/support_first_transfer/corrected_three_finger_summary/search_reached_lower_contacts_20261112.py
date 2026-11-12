"""Find lower-body contacts at the reached free-cube pose, in static IK copies.

No historical joint target is treated as a contact certificate. The cube is
fixed only during geometry fitting; candidate approaches require a separate
free dynamics replay with every existing guard.
"""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import assets,restore,Guard,motor,sha,save_json,BOUND,ticks_to_sim
from whole_hand_support import sample_model
from cube_task_frame_20261105 import aligned_top_face,orientation_error_deg,working_rotation
from upper_layer_separation_certificate_20261105 import SweepCertificate
from screen_direct_work_grip_20261112 import transition

PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
OUT=PARENT/'reached_lower_contact_search'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--lower-grip',action='store_true');args=parser.parse_args()
    out=PARENT/'lower_three_finger_contact_search' if args.lower_grip else OUT
    out.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');g=Guard(m)
    source=(PARENT/'corrected_three_finger_continuous_mu0.3/end_state.npz' if args.lower_grip
            else PARENT/'lower_pad_lift_after_clear_v2/end_state.npz')
    saved=dict(np.load(source,allow_pickle=False));d=restore(m,saved)
    cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();up=np.asarray(initial['up']);up/=np.linalg.norm(up)
    lag=d.ctrl-d.qpos[:16];sid=np.asarray((8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1))-1;T=2*np.pi/4096
    lo=np.maximum(g.qlo,(np.ceil(g.lo[sid])+1-2048)*T-lag)
    hi=np.minimum(g.qhi,(np.floor(g.hi[sid])-1-2048)*T-lag)
    rows=[];proposal=restore(m,saved)
    # RF approaches first, before MF curls across its workspace.
    for index,(f,active) in enumerate((('rf',np.arange(8,12)),('mf',np.arange(4,8)),('th',np.arange(12,16)))):
        seed=proposal.qpos.copy();tip=m.geom(f+'_tip').id;seg=np.zeros(6)
        def measure(x):
            proposal.qpos[:]=seed;proposal.qpos[active]=x;proposal.ctrl[:]=proposal.qpos[:16]+lag;mujoco.mj_forward(m,proposal)
            distance=float(mujoco.mj_geomDistance(m,proposal,cube,tip,.5,seg))*1000
            height=float((seg[:3]-C)@up*1000)
            support,own=g.contacts(proposal)
            penetration=max((-float(c.dist)*1000 for c in proposal.contact if cube in (int(c.geom1),int(c.geom2))),default=0.)
            return distance,height,max(own.values(),default=0.),max(support.values(),default=0.),penetration
        def objective(x):
            gap,height,own,support,pen=measure(x)
            return abs(gap+.01)+max(0.,height-(-12 if args.lower_grip else 6))*2+own*1500+max(0.,support-.1)*500+max(0.,pen-.8)*500+.01*np.linalg.norm(x-seed[active])
        rng=np.random.default_rng(1212+index);pop=rng.uniform(lo[active],hi[active],(40,4));pop[0]=np.clip(seed[active],lo[active],hi[active])
        fit=differential_evolution(objective,list(zip(lo[active],hi[active])),init=pop,maxiter=90,seed=1212+index,polish=True,tol=1e-6)
        gap,height,own,support,pen=measure(fit.x)
        valid=bool(abs(gap)<.15 and height<=(-11.99 if args.lower_grip else 6) and own<=1e-6 and support<=.15 and pen<=1)
        row=dict(finger=f,gap_mm=gap,contact_height_above_center_mm=height,self_mm=own,support_mm=support,cube_penetration_mm=pen,
                 static_lower_contact=valid,nfev=int(fit.nfev),goal_ticks=motor(proposal.ctrl).tolist())
        rows.append(row);print(json.dumps(row),flush=True)
        np.savez_compressed(out/(f+'_static_ik.npz'),qpos=proposal.qpos,ctrl=proposal.ctrl)
        # Keep an accepted finger; discard rejected geometry before the next fit.
        if not valid:proposal.qpos[:]=seed;proposal.ctrl[:]=seed[:16]+lag;mujoco.mj_forward(m,proposal)
        save_json(out/'progress.json',dict(rows=rows))
    target=motor(proposal.ctrl);np.savez_compressed(out/'combined_static_ik.npz',qpos=proposal.qpos,ctrl=proposal.ctrl)
    dynamics=[]
    if all(r['static_lower_contact'] for r in rows):
        for order in ('RF_MF_TH','RF_TH_MF'):
            data=restore(m,saved);guard=Guard(m);start=motor(data.ctrl);moves=transition(start,target,order);goals=np.vstack([moves,np.tile(target,(54,1))])
            count=supported=0;loaded={f:0 for f in ('rf','mf','th')};stop=[];C0=B0=None;shift=drift=0.
            face=aligned_top_face(data.geom_xmat[cube].reshape(3,3),up)['best_candidate_id']
            cert=SweepCertificate(m,guard.hand,.3);upper_clear=0;upper_failure=None
            for i,command in enumerate(goals):
                assert np.all(command>=g.lo) and np.all(command<=g.hi)
                data.ctrl[:]=ticks_to_sim(command)
                for _ in range(75):
                    mujoco.mj_step(m,data);stop=guard.check(data)
                    if np.linalg.norm(data.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop=list(stop)+['cube_translation_over80mm']
                    if i>=len(moves):
                        if C0 is None:C0=data.geom_xpos[cube].copy();B0=data.geom_xmat[cube].reshape(3,3).copy()
                        load=sample_model(m,data,guard.hand,cube);count+=1;supported+=load['supported_sample']
                        for f in loaded:
                            loaded[f]+=sum(v['normal_N'] for n,v in load['by_geom'].items() if (m.body(int(m.geom_bodyid[m.geom(n).id])).name or '').startswith(f+'_'))>.1
                        shift=max(shift,float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000));drift=max(drift,orientation_error_deg(data.geom_xmat[cube].reshape(3,3),B0))
                        if upper_failure is None:
                            lower,bad=cert.query(data,data.geom_xpos[cube],working_rotation(data.geom_xmat[cube].reshape(3,3),face))
                            if bad or lower<=.3:upper_failure=dict(sample=count,gap_mm=lower,geoms=[m.geom(int(cert.ids[j])).name for j in bad])
                            else:upper_clear+=1
                    if stop:break
                if stop:break
            passed=bool(not stop and count==4050 and supported/count>.8 and all(v/count>.8 for v in loaded.values()) and shift<2 and drift<3)
            row=dict(order=order,done=i+1,total=len(goals),stop=stop,hold_samples=count,supported_samples=supported,
                     loaded_fractions={f:v/max(1,count) for f,v in loaded.items()},hold_shift_mm=shift,hold_drift_deg=drift,
                     temporary_three_finger_hold_passed=passed,upper_certified_samples=upper_clear,upper_first_failure=upper_failure,
                     final_top=aligned_top_face(data.geom_xmat[cube].reshape(3,3),up),
                     worst=guard.worst,terminal_guard=guard.last)
            folder=out/order;folder.mkdir();np.savez_compressed(folder/'research_goals.npz',start=start,goals=goals[:i+1]);np.savez_compressed(folder/'end_state.npz',**{k:getattr(data,k) for k in BOUND},time=data.time)
            save_json(folder/'review.json',row);dynamics.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('worst','terminal_guard')}),flush=True)
    save_json(out/'review.json',dict(rows=rows,dynamic_trials=dynamics,source_sha256=sha(source),model_provenance=provenance,
              source_path=source.relative_to(ROOT).as_posix(),contact_height_threshold_mm=-12 if args.lower_grip else 6,
              cube_fixed_only_in_ik=True,source_top_face=aligned_top_face(d.geom_xmat[cube].reshape(3,3),up),
              native_guards_and_limits_unchanged=True,devices_opened=0,physical_success=False,hardware_execution_allowed=False,
              upper_space_final_alignment_and_next_rotation_certified=False))

if __name__=='__main__':main()
