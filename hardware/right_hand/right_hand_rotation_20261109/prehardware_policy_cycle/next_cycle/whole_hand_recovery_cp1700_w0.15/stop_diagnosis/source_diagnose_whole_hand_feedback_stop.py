"""Explain a local planner's rejected proposals; all branches remain simulated."""
from pathlib import Path
from collections import Counter
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,motor,ticks_to_sim,save_json,sha
from whole_hand_support import sample_model


def main():
    source=OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15/end_state.npz'
    folder=OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15/stop_diagnosis'
    if folder.exists():raise RuntimeError('Preserve prior diagnosis')
    model,initial,_,_,provenance=assets('active_ccd50');state=dict(np.load(source,allow_pickle=False))
    d=restore(model,state);guard=Guard(model);previous=motor(d.ctrl);measured=motor(d.qpos[:16]);cube=model.geom('cube').id
    proposals=[('hold',previous.copy())]
    for index in range(16):
        for sign in (-1,1):
            goal=previous.copy();goal[index]+=sign*2;proposals.append((f'motor{index+1}_{sign:+}',goal))
    proposals.append(('all_tracking_relief',previous+np.clip(np.rint(measured-previous),-2,2)))
    rng=np.random.default_rng(1700121)
    for index in range(64):proposals.append((f'coupled{index}',previous+rng.choice([-2,0,2],16)))
    rows=[];safe=[]
    for label,goal in proposals:
        if np.max(abs(goal-previous))>2:raise AssertionError('Rate changed')
        trial=restore(model,state);check=Guard(model);stop=[]
        if np.any(goal<guard.lo) or np.any(goal>guard.hi):
            rows.append(dict(proposal=label,stop=['native_goal_range'],valid=False));continue
        trial.ctrl[:]=ticks_to_sim(goal)
        for step in range(75):
            mujoco.mj_step(model,trial);stop=check.check(trial)
            if np.linalg.norm(trial.qpos[-7:-4]-initial['initial_cube_center'])>.08:
                stop=list(stop)+['cube_translation_over80mm']
            if stop:break
        rows.append(dict(proposal=label,valid=not stop,stop=stop,substeps=step+1,
            goal_ticks=goal.tolist(),last_guard=check.last,whole_hand=sample_model(model,trial,guard.hand,cube)))
        if not stop:safe.append(dict(proposal=label,goal_ticks=goal.tolist()))
    folder.mkdir();counts=Counter(reason for row in rows for reason in row['stop'])
    report=dict(status='local_feedback_proposal_rejection_diagnosis',source=source.relative_to(OUT).as_posix(),
        source_sha256=sha(source),model_provenance=provenance,proposals=len(rows),safe_proposals=safe,
        rejection_counts=dict(counts),rows=rows,original_guards_and_limits_retained=True,
        initialized_prediction_branches_not_actual_executed_goals=True,
        no_feasible_tested_proposals_is_not_a_general_infeasibility_proof=True,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_diagnose_whole_hand_feedback_stop.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(proposals=len(rows),safe=len(safe),rejection_counts=dict(counts),
        first_rejection=next((row for row in rows if row['stop']),None)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
