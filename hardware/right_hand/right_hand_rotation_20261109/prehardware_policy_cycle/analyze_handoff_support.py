"""Diagnose preserved handoff states without running or changing devices."""
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from handoff_support_wrench import contact_wrench


def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    return value


def main():
    m,_,_,_,provenance=assets('active');guard=Guard(m);rows=[]
    paths=['connected_sac_and_teacher/first_quarter_end_state.npz',
           'next_cycle/rrt0_force_feedback_root_fixed/end_state.npz',
           'next_cycle/rrt0_force_feedback_lookahead2/end_state.npz',
           'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz']
    for relative in paths:
        path=OUT/relative;d=restore(m,dict(np.load(path,allow_pickle=False)))
        w=contact_wrench(m,d,guard.hand,verify=True)
        rows.append(dict(source=relative,source_sha256=sha(path),wrench=plain(w)))
    folder=OUT/'next_cycle/handoff_refinement_20261110';folder.mkdir(exist_ok=True)
    report=dict(status='contact_wrench_convention_and_saved_state_audit',model_provenance=provenance,states=rows,
        acceptance_criteria_unchanged=True,devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'support_audit.json',report);print(json.dumps(report),flush=True)


if __name__=='__main__':main()
