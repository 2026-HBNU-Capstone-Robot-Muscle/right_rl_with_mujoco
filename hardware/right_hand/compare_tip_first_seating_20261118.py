"""Compare three bounded lower-push timings with2s tracking pauses, offline only."""
from pathlib import Path
import json
import mujoco,numpy as np
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import restore
from build_tip_first_lower_seat_20261118 import BASE,interpolate
from replay_quarter_fixed_grip_20261118 import MODEL,INITIAL,BOUND

def main():
    model=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
    z=np.load(BASE/'tip_first_lower_seat/candidate.npz',allow_pickle=False)
    prefix_count=int(np.flatnonzero(z['phases']=='transfer_thumb_wrist_and_tips')[-1])+1
    data=restore(model,dict(np.load(INITIAL,allow_pickle=False)))
    def step(d,goal,seconds=.075):
        d.ctrl[:]=ticks_to_sim(goal);until=d.time+seconds
        while d.time<until-1e-10:mujoco.mj_step(model,d)
    for q in z['goals'][:prefix_count]:step(data,q)
    seed={k:getattr(data,k).copy() for k in BOUND};seed['time']=np.array(data.time)
    start=z['goals'][prefix_count-1].copy()
    seated=z['goals'][np.flatnonzero(z['phases']=='grip_index_middle_seat')[-1]].copy()
    raised=z['goals'][np.flatnonzero(z['phases']=='grip_lower_push')[-1]].copy()
    grip=z['goals'][-1].copy()
    bases=start.copy();bases[[7,11]]=raised[[7,11]]
    half=start.copy();half[4:12]=np.rint((start[4:12]+seated[4:12])/2).astype(int);half[[7,11]]=start[[7,11]]
    variants={'coupled':[('grip_coupled_lower_push',raised)],
              'bases_first':[('grip_bases_first',bases),('grip_seat_after_base_push',raised)],
              'half_then_push':[('grip_half_curl',half),('grip_early_lower_push',raised)]}
    out=BASE/'tip_first_timing_comparison';out.mkdir(exist_ok=True);results=[]
    for name,stages in variants.items():
        d=restore(model,seed);rows=list(z['goals'][:prefix_count]);labels=list(z['phases'][:prefix_count].astype(str));previous=start.copy()
        for phase,target in stages+[('fixed_grip_close',grip)]:
            part=interpolate(previous,target);rows.extend(part);labels.extend([phase]*len(part));previous=target.copy()
        rows.extend([grip.copy()]*134);labels.extend(['fixed_grip_hold']*134)
        goals=np.array(rows,dtype=int);phases=np.array(labels,dtype='<U64')
        current=start.copy();maximum=0.;pauses=0;stop=None
        for i in range(prefix_count,len(goals)):
            goal=goals[i];began=d.time
            while float(np.max(abs(ticks_to_sim(goal)-d.qpos[:16])))>.12:
                step(d,current);pauses+=1
                if d.time-began>=2.:
                    delta=abs(ticks_to_sim(goal)-d.qpos[:16]);j=int(np.argmax(delta))
                    # The sim arrays use the official simulation ordering.
                    from hand_motor_mapping import SIM_TO_MOTOR_IDS
                    stop=dict(index=i,phase=str(phases[i]),tracking_rad=float(delta[j]),motor_id=int(SIM_TO_MOTOR_IDS[j]))
                    break
            if stop:break
            step(d,goal);current=goal.copy();maximum=max(maximum,float(np.max(abs(d.ctrl-d.qpos[:16]))))
        folder=out/name;folder.mkdir(exist_ok=True)
        np.savez_compressed(folder/'candidate.npz',start=z['start'],goals=goals,phases=phases)
        result=dict(variant=name,tracking_guard_passed=stop is None,stop=stop,maximum_completed_tracking_rad=maximum,
                    pause_steps=pauses,total_goals=len(goals),prefix_goals=prefix_count,
                    complete_hardware_guards_checked=False,physical_execution=False,
                    final_tracking_rad=float(np.max(abs(ticks_to_sim(grip)-d.qpos[:16]))))
        (folder/'tracking_review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');results.append(result)
    (out/'review.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results))

if __name__=='__main__':main()
