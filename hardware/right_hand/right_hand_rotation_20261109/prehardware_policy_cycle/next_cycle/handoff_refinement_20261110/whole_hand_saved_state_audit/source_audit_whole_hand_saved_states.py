"""Native contact queries at preserved states, without advancing physics."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from whole_hand_support import sample_model,force_on_cube,CONTRACT


def verify_contact_force_frame(model,data,cube):
    """Include external contacts when comparing with cube constraint forces."""
    body=int(model.geom_bodyid[cube]);center=data.xipos[body]
    force=np.zeros(3);torque=np.zeros(3)
    for index,contact in enumerate(data.contact):
        first,second=int(contact.geom1),int(contact.geom2)
        if cube not in (first,second):continue
        local=np.zeros(6);mujoco.mj_contactForce(model,data,index,local)
        F=force_on_cube(local[:3],contact.frame,first==cube)
        M=force_on_cube(local[3:],contact.frame,first==cube)+np.cross(contact.pos-center,F)
        force+=F;torque+=M
    jp=np.zeros((3,model.nv));jr=np.zeros((3,model.nv))
    mujoco.mj_jacBodyCom(model,data,jp,jr,body)
    address=int(model.jnt_dofadr[int(model.body_jntadr[body])])
    expected=jp.T@force+jr.T@torque
    error=float(np.max(abs(expected[address:address+6]-data.qfrc_constraint[address:address+6])))
    return dict(maximum_constraint_force_error=error,passed=error<=1e-7)


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/whole_hand_saved_state_audit'
    if folder.exists():raise RuntimeError('Preserve previous native state audit')
    model,_,_,_,provenance=assets('active_ccd50');guard=Guard(model);cube=model.geom('cube').id
    sources=[]
    for path in sorted(OUT.glob('next_cycle/staged_model_*/review.json')):
        r=json.loads(path.read_text())
        if r.get('model_kind')=='active_ccd50':sources.append(path.parent/'end_state.npz')
    sources.append(OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_schema_v2_confirmation/end_state.npz')
    rows=[]
    for path in sources:
        state=dict(np.load(path,allow_pickle=False));data=restore(model,state)
        one_state_guard=Guard(model);one_state_guard.check(data)
        rows.append(dict(source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),
            whole_hand=sample_model(model,data,guard.hand,cube),
            force_frame_check=verify_contact_force_frame(model,data,cube),
            guard_at_reconstructed_state=one_state_guard.last,
            query_is_mj_forward_at_saved_state_not_new_dynamics=True,
            contact_loss_duration_cannot_be_inferred_from_one_state=True))
    folder.mkdir()
    report=dict(status='saved_state_whole_hand_contact_queries',support_contract=CONTRACT,
        source_sha256=sha(Path(__file__)),support_source_sha256=sha(Path(__file__).with_name('whole_hand_support.py')),
        model_provenance=provenance,rows=rows,
        all_force_frame_checks_passed=all(row['force_frame_check']['passed'] for row in rows),
        initialized_queries_are_not_continuous_hold_or_rotation_success=True,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_audit_whole_hand_saved_states.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(states=len(rows),all_force_frame_checks_passed=report['all_force_frame_checks_passed'],
                          hardware_execution_allowed=False)),flush=True)


if __name__=='__main__':main()
