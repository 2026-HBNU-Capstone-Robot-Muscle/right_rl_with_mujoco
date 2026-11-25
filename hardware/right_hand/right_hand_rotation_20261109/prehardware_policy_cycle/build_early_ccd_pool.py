"""Capture actual early CCD50 states before the old and new rollouts diverge far."""
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,save_json,sha


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states'
    if folder.exists():raise RuntimeError('Preserve prior state pool')
    model,z,_,_,provenance=assets('active_ccd50');data=restore(model,z);guard=Guard(model)
    parent=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_active_ccd50/research_trace.npz'
    goals=np.load(parent,allow_pickle=False)['goals'];pool=[];stop=[]
    def record(index):pool.append(dict(index=index,time=data.time,**{k:getattr(data,k).copy() for k in BOUND}))
    record(0)
    for index,goal in enumerate(goals[:3600]):
        if np.max(abs(goal-motor(data.ctrl)))>2:raise AssertionError('Rate exceeded')
        data.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):
            mujoco.mj_step(model,data);stop=guard.check(data)
            if stop:break
        if (index+1)%100==0:record(index+1)
        if stop:break
    reference=np.load(OUT/'next_cycle/handoff_refinement_20261110/ccd_replay50/state_pool.npz',allow_pickle=False)
    pos=int(np.flatnonzero(reference['indices']==3600)[0])
    diffs={k:float(np.max(abs(getattr(data,k)-reference[k][pos]))) if reference[k][pos].size else 0. for k in BOUND}
    if stop or index+1!=3600 or max(diffs.values())!=0:raise AssertionError('Early pool failed source replay')
    folder.mkdir()
    np.savez_compressed(folder/'state_pool.npz',indices=[r['index'] for r in pool],time=[r['time'] for r in pool],
        **{k:np.array([r[k] for r in pool]) for k in BOUND})
    save_json(folder/'review.json',dict(status='actual_early_CCD50_parent_states',goals=3600,
        sampled_states=len(pool),source_goals_sha256=sha(parent),model_provenance=provenance,
        source3600_end_max_abs=diffs,state_pool_sha256=sha(folder/'state_pool.npz'),
        no_state_reset_after_initial_placement=True,not_successful_demonstrations=True,
        devices_opened=0,hardware_execution_allowed=False))
    print(json.dumps(dict(states=len(pool),source3600_end_max_abs=diffs)),flush=True)


if __name__=='__main__':main()
