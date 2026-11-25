"""Measure native cached-pose sampling differences in the new repair interface."""
import numpy as np
import mujoco
from ccd_repair_env import CCDRepair
from native_cycle import OUT,motor,ticks_to_sim,TICK,save_json,sha
from export_and_evaluate import Actor


def main():
    env=CCDRepair();rows=[]
    actor_paths=[OUT/'sac_current_contract_v4_20k/export_000000010256/actor.npz']
    actor_paths+=sorted(OUT.glob('sac_ccd50_*/export_*/actor.npz'))
    actors=[(path,Actor(path)) for path in actor_paths]
    for index in (3000,3400,3600):
        pos=int(np.flatnonzero(env.pool_indices==index)[0]);m=env.model;d=mujoco.MjData(m)
        for key in env.slices:getattr(d,key)[:]=env.pool[key][pos]
        d.time=float(env.pool['time'][pos]);mujoco.mj_forward(m,d)
        obs=env.observe(m,d,index);action=actors[0][1].action(obs)
        goal=np.clip(motor(d.ctrl)+np.clip(motor(ticks_to_sim(env.reference[index])+.01*action)-motor(d.ctrl),-2,2),env.lo,env.hi)
        prior=d.ctrl.copy();d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):mujoco.mj_step(m,d)
        b1=env._builder(obs);b2=env._builder(obs)
        for builder in (b1,b2):builder.set_last_act(np.clip((d.ctrl-prior)/.2,-1,1))
        trained=env.observe(m,d,index+1,b1)
        cube=m.geom('cube').id;quat=np.empty(4);mujoco.mju_mat2Quat(quat,d.geom_xmat[cube])
        measured=(np.rint(d.qpos[:16]/TICK+2048)-2048)*TICK
        palm=d.sensordata[m.sensor_adr[m.sensor('palm_position').id]:][:3]
        deployed=np.r_[b2.build(measured,d.ctrl,palm,d.geom_xpos[cube],quat,env.goal_quat),
            (index+1)/max(1,len(env.reference)-1),float(index+1>=len(env.reference)-134)].astype(np.float32)
        differences=[]
        for path,actor in actors:
            a,b=actor.action(trained),actor.action(deployed)
            def next_goal(action):
                p=motor(ticks_to_sim(env.reference[index+1])+.01*action)
                return np.clip(goal+np.clip(p-goal,-2,2),env.lo,env.hi)
            differences.append(dict(actor=path.relative_to(OUT).as_posix(),sha256=sha(path),
                maximum_action_difference=float(np.max(abs(a-b))),
                different_next_joint_goals=int(np.count_nonzero(next_goal(a)!=next_goal(b)))))
        rows.append(dict(start=index,cube_translation_cache_difference_mm=float(np.linalg.norm(d.qpos[-7:-4]-d.geom_xpos[cube])*1000),
            maximum_observation_difference=float(np.max(abs(trained-deployed))),actors=differences))
    save_json(OUT/'next_cycle/handoff_refinement_20261110/observation_timing_audit.json',dict(
        status='native_cached_vs_postintegrated_pose_sampling_audit',rows=rows,
        scope='three native one-step probes; not full closed-loop provider equivalence or physical camera validation',
        all_compared_next_goals_match=all(a['different_next_joint_goals']==0 for r in rows for a in r['actors']),
        devices_opened=0,hardware_execution_allowed=False))
    print(str(OUT/'next_cycle/handoff_refinement_20261110/observation_timing_audit.json'),flush=True)


if __name__=='__main__':main()
