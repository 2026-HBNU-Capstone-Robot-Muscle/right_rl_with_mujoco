"""Screen the new handover, using the previous model grip as a hypothesis only."""
from pathlib import Path
import json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from research_rotation_body_hold_20261104 import inspect_for,restore
from grip_to_rotation_plan_20261118 import BASE,DATA

ROOT=Path(__file__).resolve().parent
MODEL=ROOT/'right_hand_rotation_20261105/canonical_body_grip/torque_cap_sensitivity/cap0.15/model.mjb'
SEED=ROOT/'right_hand_rotation_20261118/quarter_turn_fixed_grip/tip_first_lower_seat/model_end.npz'

def main(base=None,recorded_goal=None):
    output=BASE if base is None else Path(base)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()});d=mujoco.MjData(m)
    model_extensions=[]
    if recorded_goal is not None:
        q=ticks_to_sim(recorded_goal)
        for sim,value in enumerate(q):
            joint=int(np.flatnonzero(m.jnt_qposadr==sim)[0])
            before=m.jnt_range[joint].copy();control=m.actuator_ctrlrange[sim].copy()
            m.jnt_range[joint]=[min(before[0],value),max(before[1],value)]
            m.actuator_ctrlrange[sim]=[min(control[0],value),max(control[1],value)]
            if not np.array_equal(before,m.jnt_range[joint]):
                model_extensions.append(dict(sim_index=sim,old_joint_range_rad=before.tolist(),new_joint_range_rad=m.jnt_range[joint].tolist()))
        mujoco.mj_setConst(m,mujoco.MjData(m))
    z=np.load(output/'candidate.npz',allow_pickle=False);goals=z['goals'];labels=z['phases'];_,_,check=inspect_for(m)
    old=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)
    turn_start=int(np.flatnonzero(old['phases']=='stage_7_horizontal_yaw')[0])
    endpoints=[z['start'],old['goals'][turn_start-1],old['goals'][-1]]
    baseline=[{},{}]
    def geometry(q):
        d.qpos[:16]=ticks_to_sim(q);mujoco.mj_fwdPosition(m,d);return check(d)
    for q in endpoints:
        for dst,values in zip(baseline,geometry(q)):
            for k,v in values.items():dst[k]=max(dst.get(k,0.),float(v))
    measured_geometry=None if recorded_goal is None else geometry(recorded_goal)
    worst=[{},{}];extra=[0.,0.]
    for q in goals:
        for j,values in enumerate(geometry(q)):
            for k,v in values.items():worst[j][k]=max(worst[j].get(k,0.),float(v));extra[j]=max(extra[j],float(v)-baseline[j].get(k,0.))
    dynamic=restore(m,dict(np.load(SEED,allow_pickle=False)));cube=m.geom('cube').id
    hand,support,_=inspect_for(m);current=z['start'];stop=None;max_track=0.;pauses=0;phase_ends=[]
    palm=m.body('palm').id
    initial_cube=dynamic.geom_xpos[cube].copy()
    # A diagnostic only: no cube relocation or external force is applied.
    def cube_state(index,phase):
        contacts=[]
        for c in dynamic.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube in (a,b) and c.dist<.0004:
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:contacts.append(m.geom(other).name)
        return dict(index=int(index),phase=str(phase),cube_world_m=dynamic.geom_xpos[cube].tolist(),
            cube_from_palm_mm=((dynamic.geom_xpos[cube]-dynamic.xpos[palm])@dynamic.xmat[palm].reshape(3,3)*1000).tolist(),
            displacement_from_initial_mm=float(np.linalg.norm(dynamic.geom_xpos[cube]-initial_cube)*1000),
            hand_contacts=sorted(set(contacts)))
    def step(goal):
        dynamic.ctrl[:]=ticks_to_sim(goal);until=dynamic.time+.075
        while dynamic.time<until-1e-10:mujoco.mj_step(m,dynamic)
    for i,q in enumerate(goals):
        began=dynamic.time
        while float(np.max(abs(ticks_to_sim(q)-dynamic.qpos[:16])))>.12:
            step(current);pauses+=1
            if dynamic.time-began>=2.:
                stop=dict(index=i,phase=str(labels[i]),reason='tracking did not recover within2s');break
        if stop:break
        step(q);current=q.copy();max_track=max(max_track,float(np.max(abs(dynamic.ctrl-dynamic.qpos[:16]))))
        if i+1==len(goals) or labels[i+1]!=labels[i]:phase_ends.append(cube_state(i,labels[i]))
    contacts=[]
    for c in dynamic.contact:
        a,b=int(c.geom1),int(c.geom2)
        if cube in (a,b) and c.dist<.0004:
            other=b if a==cube else a
            if int(m.geom_bodyid[other]) in hand:contacts.append(m.geom(other).name)
    r=dict(kind='grip_to_turn_static_and_nominal_dynamic_diagnostic',new_self_over_endpoint_baseline_mm=extra[1],
        new_support_over_endpoint_baseline_mm=extra[0],self_worst_mm=worst[1],support_worst_mm=worst[0],
        tracking_stop=stop,tracking_pause_steps=pauses,maximum_completed_tracking_rad=max_track,
        final_hand_contacts=sorted(set(contacts)),cube_pose_is_previous_model_hypothesis=True,
        phase_end_cube_diagnostics=phase_ends,cube_state_reset_during_path=False,external_cube_forces=False,
        actual_grip_to_turn_transfer_verified=False,complete_motor_guards_checked=False,
        exact_measured_target_model_range_extensions=model_extensions,
        geometry_at_measured_target=measured_geometry,
        physical_execution=False)
    (output/'model_review.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(r))

if __name__=='__main__':main()
