"""Compare individual and net finger direction on the blocked reached state."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from directed_contact_state import finger_yaw_moments

def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';source=base/'robust_mu025_feedback_bounded_fit_v13/force1.5/end_state.npz';folder=base/'reposition_then_second_rotation_20261111/finger_moment_admission_audit'
    if folder.exists():raise RuntimeError('Preserve actual contact audit')
    m,initial,*_=assets('active_ccd50');m.geom_friction[:,0]=.25;d=restore(m,dict(np.load(source,allow_pickle=False)));g=Guard(m);cube=m.geom('cube').id;rows=[];stop=[]
    rows.append(dict(after_ms=0,**finger_yaw_moments(m,d,cube,'mf',initial['up'])))
    for i in range(75):
        mujoco.mj_step(m,d);stop=g.check(d)
        row=finger_yaw_moments(m,d,cube,'mf',initial['up'])
        if i in (0,74) or row['strict_any_opposed_rejected'] or stop:rows.append(dict(after_ms=i+1,native_stop=stop,**row))
        if stop:break
    folder.mkdir();r=dict(status='actual_opposed_and_net_MF_contact_moments',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),rows=rows,unchanged_command_75ms_native_stop=stop,individual_opposed_support_not_force_closure_failure=True,normal_moment_is_local_direction_not_total_task_success=True,devices_opened=0,hardware_execution_allowed=False);save_json(folder/'review.json',r)
    print(json.dumps(dict(status=r['status'],first_opposed=next((r for r in rows if r['strict_any_opposed_rejected']),None),stop=stop)),flush=True)

if __name__=='__main__':main()
