"""Replay the preserved full rotation, then candidate transfers, without cube resets."""
from pathlib import Path
import argparse,hashlib,json
import mujoco,numpy as np
from research_rotation_body_hold_20261104 import restore,inspect_for
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=MODEL.parent/'initial_state.npz'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
BOUND=('qpos','qvel','ctrl','act','qacc_warmstart')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--variants',default='direct,receiving,staged')
    ap.add_argument('--tracking-guard',action='store_true')
    ap.add_argument('--guard-start',type=int,default=1417);args=ap.parse_args()
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    initial=dict(np.load(INITIAL,allow_pickle=False));d=restore(m,initial)
    cube=m.geom('cube').id;hand,support,check=inspect_for(m)
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy()
    up=initial.get('up',-m.opt.gravity/np.linalg.norm(m.opt.gravity))
    desired=upright_yaw_target(B0,up,90.)
    first=np.load(FIRST,allow_pickle=False)['goals']
    def step(data,goal):
        data.ctrl[:]=ticks_to_sim(goal);until=data.time+.075
        while data.time<until-1e-10:mujoco.mj_step(m,data)
    def summary(data):
        names=[];external=[]
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b) or c.dist>=.0004:continue
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:names.append(m.geom(other).name)
            elif other in support:external.append(m.geom(other).name)
        return dict(cube_orientation_error_degrees=orientation_error_deg(data.geom_xmat[cube].reshape(3,3),desired),
            center_change_mm=float(np.linalg.norm(data.geom_xpos[cube]-C0)*1000),
            hand_contact_names=sorted(set(names)),external_contacts=sorted(set(external)),
            joint_tracking_rad=float(max(abs(data.ctrl-data.qpos[:16]))))
    for goal in first:step(d,goal)
    prefix={k:getattr(d,k).copy() for k in BOUND};prefix['time']=np.array(d.time)
    np.savez_compressed(BASE/'preserved_rotation_model_end.npz',**prefix)
    prefix_summary=summary(d)
    results=[]
    for name in args.variants.split(','):
        data=restore(m,prefix);candidate=BASE/name/'candidate.npz'
        z=np.load(candidate,allow_pickle=False);assert np.array_equal(z['goals'][:1417],first)
        maximum_tracking=0.;maximum_support={};maximum_self={};trace=[]
        from goal_aware_progress_guard_20261025 import GoalAwareProgress
        progress=GoalAwareProgress();current=first[-1].copy();guard_stop=None;pause_steps=0
        for i,goal in enumerate(z['goals'][1417:]):
            if args.tracking_guard and 1417+i>=args.guard_start:
                began=data.time
                while True:
                    try:
                        progress.check(str(z['phases'][1417+i]),np.flatnonzero(z['goals'][-1]!=current),
                            current,sim_to_ticks(data.qpos[:16]),data.time)
                    except RuntimeError as exc:
                        guard_stop=dict(index=1417+i,phase=str(z['phases'][1417+i]),error=str(exc));break
                    if float(max(abs(ticks_to_sim(goal)-data.qpos[:16])))<=.12:break
                    if data.time-began>=2.:
                        guard_stop=dict(index=1417+i,phase=str(z['phases'][1417+i]),error='Tracking did not recover within2s');break
                    step(data,current);pause_steps+=1
                if guard_stop:break
            step(data,goal)
            current=goal.copy()
            if not np.isfinite(data.qpos).all():raise RuntimeError('Nonfinite model state')
            sp,own=check(data)
            for key,value in sp.items():maximum_support[key]=max(maximum_support.get(key,0.),float(value))
            for key,value in own.items():maximum_self[key]=max(maximum_self.get(key,0.),float(value))
            maximum_tracking=max(maximum_tracking,float(max(abs(data.ctrl-data.qpos[:16]))))
            if i%40==0:trace.append(dict(index=i,phase=str(z['phases'][1417+i]),**summary(data)))
        result=dict(variant=name,source_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
            model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
            initial_sha256=hashlib.sha256(INITIAL.read_bytes()).hexdigest(),
            preserved_rotation_model=prefix_summary,final=summary(data),
            maximum_tracking_rad=maximum_tracking,maximum_support_mm=maximum_support,maximum_self_mm=maximum_self,
            exact_fixed_goal_written=bool(np.array_equal(data.ctrl,ticks_to_sim(z['goals'][-1]))),
            no_state_reset_between_rotation_and_grip=True,physical_execution=False,hardware_execution_ready=False,
            tracking_and_progress_guards_enabled=args.tracking_guard,guard_stop=guard_stop,
            guarded_goal_start_index=args.guard_start if args.tracking_guard else None,
            tracking_pause_steps=pause_steps,complete_motor_guards_checked=False,
            model_angle_is_not_the_observed_real_rotation_angle=True,trace=trace)
        folder=BASE/name
        np.savez_compressed(folder/'model_end.npz',**{k:getattr(data,k) for k in BOUND},time=np.array(data.time))
        (folder/'loaded_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        results.append({k:v for k,v in result.items() if k not in ('trace','initial_sha256','model_sha256','source_sha256')})
    (BASE/('loaded_summary.json' if args.variants=='direct,receiving,staged' else 'transfer_extension_summary.json')).write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results))


if __name__=='__main__':main()
