"""Model-only thumb withdrawal/reset/recontact while the palm and fingers support the cube."""
from pathlib import Path
import argparse,hashlib,json
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from research_rotation_body_hold_20261104 import restore

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261117/user_measured_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=BASE/'two_small_turns_model_probe/end.npz'


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--open-ticks',type=int,choices=(-224,-160,-96,96,160,224),default=160)
    ap.add_argument('--opening-id',type=int,choices=(2,3),default=3)
    ap.add_argument('--restore-axis',action='store_true')
    ap.add_argument('--cycles',type=int,choices=(0,3,9),default=0)
    args=ap.parse_args()
    opening_index=args.opening_id-1
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    d=restore(m,dict(np.load(INITIAL,allow_pickle=False)));cube=m.geom('cube').id
    C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    axis=d.xmat[m.body('palm').id].reshape(3,3)[:,2].copy()
    goal=sim_to_ticks(d.ctrl);start=goal.copy();trace=[];rows=[];phases=[];stop=None
    def advance(target,phase):
        nonlocal goal,stop
        steps=max(1,int(np.ceil(max(abs(target-goal))/2)));begin=goal.copy()
        for k in range(1,steps+1):
            goal=np.rint(begin+(target-begin)*k/steps).astype(int)
            d.ctrl[:]=ticks_to_sim(goal);until=d.time+.060
            while d.time<until-1e-10:mujoco.mj_step(m,d)
            rows.append(goal.copy());phases.append(phase)
            forces={};names=[]
            for ci,c in enumerate(d.contact):
                a,b=int(c.geom1),int(c.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a;name=m.geom(other).name or ''
                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                if force[0]>.02:names.append(name)
                role=(m.body(int(m.geom_bodyid[other])).name or '').split('_')[0]
                forces[role]=forces.get(role,0)+float(force[0])
            shift=float(np.linalg.norm(d.geom_xpos[cube]-C)*1000)
            change=Rotation.from_matrix(d.geom_xmat[cube].reshape(3,3)@B.T).as_rotvec()
            tracking=float(max(abs(d.ctrl-d.qpos[:16])))
            trace.append(dict(phase=phase,time=float(d.time),center_shift_mm=shift,
                yaw_change_degrees=float(np.degrees(change@axis)),group_force_n=forces,
                contact_names=names,tracking_rad=tracking))
            if shift>20 or tracking>.18:
                stop='model shift/tracking limit';return False
        return True
    target=goal.copy();target[opening_index]+=args.open_ticks
    if advance(target,'thumb_withdraw'):
        target=goal.copy();target[3]=2988
        if advance(target,'thumb_root_reset'):
            while goal[opening_index]>start[opening_index]-48 and not stop:
                target=goal.copy();target[opening_index]-=2
                if not advance(target,'thumb_recontact'):break
                if trace[-1]['group_force_n'].get('th',0)>.4:break
            if args.restore_axis and not stop:
                target=goal.copy();target[opening_index]=start[opening_index]
                advance(target,'thumb_axis_restore')
            if not stop:
                for _ in range(34):
                    if not advance(goal.copy(),'regrasp_hold'):break
    for cycle in range(args.cycles):
        if stop:break
        for index,value,name in [(3,int(start[3]),'turn'),(opening_index,int(start[opening_index]+args.open_ticks),'axis_sweep'),
                                  (3,2988,'root_reset'),(opening_index,int(start[opening_index]),'axis_restore')]:
            target=goal.copy();target[index]=value
            if not advance(target,f'cycle{cycle+1}_{name}'):break
        if not stop:
            for _ in range(17):
                if not advance(goal.copy(),f'cycle{cycle+1}_hold'):break
    out=BASE/('thumb_reset_open'+str(args.open_ticks)+('_id2' if args.opening_id==2 else '')+('_restore' if args.restore_axis else '')+
              ('_cycles'+str(args.cycles) if args.cycles else ''));out.mkdir(exist_ok=True)
    np.savez_compressed(out/'research_goals.npz',goals=np.array(rows),phases=np.array(phases),start_goal=start)
    np.savez_compressed(out/'end.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,
                        qacc_warmstart=d.qacc_warmstart,time=np.array(d.time))
    ends=[row for i,row in enumerate(trace) if i==len(trace)-1 or trace[i+1]['phase']!=row['phase']]
    report=dict(kind='thumb_reset_with_palm_support_model_diagnostic',open_ticks=args.open_ticks,
        opening_motor_id=args.opening_id,
        axis_restored=args.restore_axis,
        cycles_requested=args.cycles,
        withdrawal_force_below_0p02n=any(r['group_force_n'].get('th',0)<.02 for r in trace if r['phase']=='thumb_withdraw'),
        source_sha256=hashlib.sha256(INITIAL.read_bytes()).hexdigest(),phase_ends=ends,stop_reason=stop,
        max_center_shift_mm=max(r['center_shift_mm'] for r in trace),
        final_yaw_change_degrees=trace[-1]['yaw_change_degrees'],goal_count=len(rows),
        support_does_not_require_all_fingers_contacting=True,physical_cube_pose_measured=False,
        physical_motion_executed=False,executable_candidate=False,trace=trace)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','phase_ends')}))
    print('last_phase',json.dumps(ends[-1]))


if __name__=='__main__':main()
