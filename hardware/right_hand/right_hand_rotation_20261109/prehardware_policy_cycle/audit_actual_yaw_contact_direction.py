"""Check actual rather than proposed MF yaw moment arms after contact arrival."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,ticks_to_sim,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg,upright_yaw_target


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';root=base/'reposition_then_second_rotation_20261111';folder=root/'actual_contact_direction_audit'
    if folder.exists():raise RuntimeError('Preserve direction audit')
    source=base/'robust_mu025_reposition_MF_contact_arrival_v1/axis0_sign+1_force0/end_state.npz';parent=base/'robust_mu025_reposition_MF_pressure_v1/force0.5/research_goals.npz'
    m,initial,_,_,_=assets('active_ccd50');m.geom_friction[:,0]=.25;d=restore(m,dict(np.load(source,allow_pickle=False)));cube=m.geom('cube').id;tip=m.geom('mf_tip').id;g=Guard(m);rows=[]
    def sample(step):
        points=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,cube,tip,.2,points));n=points[3:]-points[:3]
        contacts=[c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
        if contacts:
            c=min(contacts,key=lambda c:c.dist);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);point=c.pos
        else:point=points[:3]
        n/=np.linalg.norm(n);lever=float(np.cross(point-d.geom_xpos[cube],-n)@initial['up']*1000)
        rows.append(dict(step=step,tip_gap_mm=gap*1000,positive_yaw_lever_mm=lever,actual_contact_count=len(contacts),error180_deg=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],180))))
    sample(0)
    for i,cmd in enumerate(np.load(parent,allow_pickle=False)['goals'][:200],1):
        d.ctrl[:]=ticks_to_sim(cmd)
        for _ in range(75):
            mujoco.mj_step(m,d);assert not g.check(d),g.last
        if i in (10,25,50,100,150,200):sample(i)
    folder.mkdir();(folder/'source_audit_actual_yaw_contact_direction.py').write_bytes(Path(__file__).read_bytes());r=dict(status='actual_contact_direction_after_arrival',source=source.relative_to(OUT).as_posix(),source_sha256=sha(source),replayed_pressure_goals=parent.relative_to(OUT).as_posix(),replayed_pressure_goals_sha256=sha(parent),rows=rows,static_positive_lever_not_substituted_for_actual_arrival=True,devices_opened=0,hardware_execution_allowed=False);save_json(folder/'review.json',r);print(json.dumps(r),flush=True)


if __name__=='__main__':main()
