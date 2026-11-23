"""Preserved full turn -> supported lift/back/seat proposal -> fixed real grip.

Offline only. Desired cube poses belong exclusively to the IK proposal data.
The integrated cube is never reset, welded or given an external force.
"""
from pathlib import Path
import hashlib,json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
from research_rotation_body_hold_20261104 import restore,propose,motor,native,inspect_for
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from build_quarter_fixed_grip_20261118 import interpolate

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip'
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
INITIAL=MODEL.parent/'initial_state.npz'
PREFIX=BASE/'preserved_rotation_model_end.npz'
FIRST=ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz'
REFERENCE=ROOT/'right_hand_rotation_20261117/user_measured_grip/reference.json'
SEATING=ROOT/'right_hand_rotation_20261117/user_measured_grip/model_hold_probe/end.npz'
BOUND=('qpos','qvel','ctrl','act','qacc_warmstart')


def main():
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    metadata=dict(np.load(INITIAL,allow_pickle=False));d=restore(m,dict(np.load(PREFIX,allow_pickle=False)))
    cube=m.geom('cube').id;hand,support,_=inspect_for(m)
    grip=np.array(json.loads(REFERENCE.read_bytes())['holding_goal_ticks_readonly'],dtype=int)
    lo,hi,_,_=native(m);hi[10]=max(hi[10],grip[10])
    first=np.load(FIRST,allow_pickle=False);previous=motor(d.ctrl)
    assert np.array_equal(previous,first['goals'][-1])
    C0=d.geom_xpos[cube].copy();B0=d.geom_xmat[cube].reshape(3,3).copy();up=metadata['up']
    desired=upright_yaw_target(metadata['initial_cube_rotation'],up,90.)
    final_center=np.load(SEATING,allow_pickle=False)['qpos'][-7:-4]
    delta=final_center-C0;height=float(delta@up)
    horizontal=delta-up*height
    lift=max(height+.008,.012)
    target_centers=[C0+up*lift,C0+up*lift+horizontal,final_center+up*.001]
    rows=[];phases=[];trace=[];records=[];stop=None;maximum_tracking=0.
    def apply(goals,phase):
        nonlocal previous,stop,maximum_tracking
        for goal in goals:
            if np.any(goal<lo) or np.any(goal>hi) or max(abs(goal-previous))>2:
                stop='native/scoped step mismatch';return False
            d.ctrl[:]=ticks_to_sim(goal);until=d.time+.075
            while d.time<until-1e-10:mujoco.mj_step(m,d)
            error=float(max(abs(d.ctrl-d.qpos[:16])));maximum_tracking=max(maximum_tracking,error)
            rows.append(goal.copy());phases.append(phase);previous=goal.copy()
            if len(rows)%20==0:
                trace.append(dict(index=len(rows),phase=phase,time=float(d.time),
                    cube_center=d.geom_xpos[cube].tolist(),orientation_change_degrees=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0),tracking_rad=error))
            if error>.18 or not np.isfinite(d.qpos).all():
                stop='model tracking diagnostic';return False
        return True
    # Increase the two existing fingertip normal preloads through joint goals,
    # never by applying an object force or converting0.15Nm to450mA.
    delta_tau=np.zeros(16)
    for name in ('mf_tip','th_tip'):
        gid=m.geom(name).id
        contacts=[]
        for ci,c in enumerate(d.contact):
            if {int(c.geom1),int(c.geom2)}!={cube,gid}:continue
            f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);contacts.append((float(f[0]),c))
        if not contacts:stop='carry contact unavailable';break
        existing,c=max(contacts,key=lambda item:item[0]);normal=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
        jac=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jac,None,c.pos,int(m.geom_bodyid[gid]))
        delta_tau-=jac[:,:16].T@normal*max(0.,1.8-existing)
    if not stop:
        target=motor(d.ctrl+delta_tau/m.actuator_gainprm[:,0])
        target[[i for i in range(16) if i not in (0,1,2,3,8,9,10,11)]]=previous[[i for i in range(16) if i not in (0,1,2,3,8,9,10,11)]]
        apply(interpolate(previous,target),'carry_preload')
    for phase,center in zip(('lift_to_palm_height','carry_over_palm','seat_on_palm'),target_centers):
        if stop:break
        state=dict(metadata)
        state.update({k:getattr(d,k).copy() for k in BOUND});state['time']=np.array(d.time)
        translation=(center-d.geom_xpos[cube])*1000
        try:goals,detail,failure=propose(m,state,0.,0.,3.,translation_mm=translation)
        except Exception as exc:goals,detail,failure=None,[],str(exc)
        records.append(dict(phase=phase,translation_mm=translation.tolist(),failure=failure,detail=detail))
        if goals is None:stop='carry proposal: '+str(failure);break
        if not apply(goals,phase):break
    if not stop:
        if apply(interpolate(previous,grip),'fixed_grip_close'):
            apply(np.tile(grip,(134,1)),'fixed_grip_hold')
    contacts=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if cube in (a,b) and c.dist<.0004:
            g=b if a==cube else a
            if int(m.geom_bodyid[g]) in hand:contacts.append(m.geom(g).name)
    final_B=d.geom_xmat[cube].reshape(3,3)
    out=BASE/'cartesian_carry';out.mkdir(exist_ok=True)
    full=np.vstack([first['goals'],np.array(rows,dtype=int)]) if rows else first['goals']
    full_phases=np.r_[first['phases'].astype('<U64'),np.array(phases,dtype='<U64')]
    np.savez_compressed(out/'research_candidate.npz',start=first['start'],goals=full,phases=full_phases)
    np.savez_compressed(out/'end.npz',**{k:getattr(d,k) for k in BOUND},time=np.array(d.time))
    report=dict(kind='preserved_full_turn_lift_carry_then_fixed_grip',first1417_exactly_preserved=True,
        stop_reason=stop,carry_center_delta_mm=(delta*1000).tolist(),lift_mm=lift*1000,
        cube_orientation_error_degrees=orientation_error_deg(final_B,desired),
        cube_orientation_change_after_rotation_degrees=orientation_error_deg(final_B,B0),
        final_hand_contacts=sorted(set(contacts)),maximum_tracking_rad=maximum_tracking,
        exact_fixed_grip_command_reached=bool(np.array_equal(motor(d.ctrl),grip)),
        final_joint_tracking_rad=float(max(abs(d.ctrl-d.qpos[:16]))),
        total_goals=len(full),records=records,trace=trace,
        integrated_cube_reset=False,external_cube_forces=False,
        seating_center_is_an_unmeasured_model_hypothesis=True,
        torque_cap_is_not_a_motor_current_conversion=True,
        physical_execution=False,hardware_execution_ready=False,
        first_source_sha256=hashlib.sha256(FIRST.read_bytes()).hexdigest(),
        fixed_grip_reference_sha256=hashlib.sha256(REFERENCE.read_bytes()).hexdigest())
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('trace','records')}))
    if records:print('last_proposal',json.dumps({k:v for k,v in records[-1].items() if k!='detail'}))


if __name__=='__main__':main()
