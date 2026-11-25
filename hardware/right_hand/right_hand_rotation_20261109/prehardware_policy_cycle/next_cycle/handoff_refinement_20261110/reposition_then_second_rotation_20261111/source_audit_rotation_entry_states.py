"""Compare reached low-friction support with the proven next-turn entry state."""
from pathlib import Path
import json
import numpy as np
from native_cycle import OUT, assets, restore, Guard, motor, save_json, sha
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'reposition_then_second_rotation_20261111'
    if root.exists():raise RuntimeError('Preserve entry comparison')
    source=base/'robust_cycle_preparation_20261111/low_friction_first_hold_state_v4/end_state.npz'
    target=base/'initial_to_reconfigured_first90/end_state.npz'
    m,_,_,_,provenance=assets('active_ccd50');m.geom_friction[:,0]=.25;g=Guard(m);cube=m.geom('cube').id;states=[]
    for label,path in (('actual_low_friction_first_hold',source),('nominal_successful_second_turn_entry_reference',target)):
        d=restore(m,dict(np.load(path,allow_pickle=False)))
        states.append(dict(label=label,path=path.relative_to(OUT).as_posix(),sha256=sha(path),cube_center_world_m=d.geom_xpos[cube].tolist(),cube_rotation_world=d.geom_xmat[cube].reshape(3,3).tolist(),
            command_motor_ticks=motor(d.ctrl).tolist(),actual_motor_ticks=motor(d.qpos[:16]).tolist(),restored_model_support=sample_model(m,d,g.hand,cube)))
    a,b=states;delta=np.asarray(b['cube_center_world_m'])-a['cube_center_world_m'];q=np.asarray(b['command_motor_ticks'])-a['command_motor_ticks']
    report=dict(status='actual_and_reference_rotation_entry_states_compared',model_provenance=provenance,states=states,
        desired_cube_translation_world_mm=(delta*1000).tolist(),cube_center_difference_mm=float(np.linalg.norm(delta)*1000),
        cube_orientation_difference_deg=orientation_error_deg(np.asarray(a['cube_rotation_world']),np.asarray(b['cube_rotation_world'])),
        desired_motor_command_difference_ticks=q.tolist(),maximum_motor_command_difference_ticks=float(max(abs(q))),
        nominal_reference_is_not_an_actual_reached_low_friction_state=True,cube_or_hand_never_teleported=True,
        devices_opened=0,hardware_execution_allowed=False)
    root.mkdir();(root/'source_audit_rotation_entry_states.py').write_bytes(Path(__file__).read_bytes());save_json(root/'entry_state_comparison.json',report)
    print(json.dumps({k:report[k] for k in ('desired_cube_translation_world_mm','cube_center_difference_mm','cube_orientation_difference_deg','maximum_motor_command_difference_ticks','desired_motor_command_difference_ticks')}),flush=True)


if __name__=='__main__':main()
