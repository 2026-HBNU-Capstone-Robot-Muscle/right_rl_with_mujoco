"""Compare the two planning costs against actual stored model geometry."""
import json
import time
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from convex_pair_gap import ConvexPairGap
from planning_pair_gap import PlanningPairGap


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';model,_,_,_,_=assets('active_ccd50')
    pairs=[('mf_tip',f'palm_collision_{i}') for i in (1,4,6,8)]
    pairs += [('mf_tip','rf_md_collision_5'),('cadtip_mf_rf_mf_md_0','cadtip_mf_rf_rf_md_0'),('cadtip_mf_rf_mf_px_0','cadtip_mf_rf_rf_md_0')]
    ids=[(model.geom(a).id,model.geom(b).id) for a,b in pairs];geoms=sorted({g for p in ids for g in p})
    complete=ConvexPairGap(model,geoms);bounded=PlanningPairGap(model,geoms)
    paths=[base/'RF_clear_work_grip_joined_suffix/end_state.npz',OUT/'next_cycle/whole_hand_recovery_cp1700_w0.5_clear_work_grip_reference/start_state.npz']
    rows=[]
    for path in paths:
        data=restore(model,dict(np.load(path,allow_pickle=False)))
        begin=time.perf_counter();exact=[complete.gap(data,a,b)*1000 for a,b in ids];exact_seconds=time.perf_counter()-begin
        begin=time.perf_counter();fast=[bounded.gap(data,a,b)*1000 for a,b in ids];fast_seconds=time.perf_counter()-begin
        rows.append(dict(source=path.relative_to(OUT).as_posix(),complete_pair_set_seconds=exact_seconds,bounded_pair_set_seconds=fast_seconds,
            complete_axis_gaps_mm=exact,bounded_axis_gaps_mm=fast,native_guard=Guard(model).check(data)))
    result=dict(status='bounded_planning_axis_cost_profile',rows=rows,pairs=pairs,bounded_source_sha256=sha(OUT/'planning_pair_gap.py'),
        bounded_cost_is_not_native_collision_or_final_clearance_certificate=True,native_model_and_guards_unchanged=True,devices_opened=0)
    save_json(base/'planning_pair_gap_benchmark.json',result);print(json.dumps(result),flush=True)


if __name__=='__main__':main()
