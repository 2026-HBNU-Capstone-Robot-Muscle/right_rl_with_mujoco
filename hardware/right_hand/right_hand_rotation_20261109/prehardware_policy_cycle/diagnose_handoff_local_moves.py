"""Independent bounded-move diagnostics at the preserved closing plateau."""
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,motor,ticks_to_sim,SIM_TO_MOTOR_IDS,save_json,sha
from convex_distance_check_20261104 import CheckedDistance
from handoff_support_wrench import contact_wrench


def main():
    source=OUT/'next_cycle/handoff_integer_endpoint_close_v4/end_state.npz';z=dict(np.load(source,allow_pickle=False))
    m,_,_,_,provenance=assets('active');g=Guard(m);cube=m.geom('cube').id
    geometry=CheckedDistance(m,cube,[m.geom(n).id for n in ('rf_tip','th_tip')]);previous=motor(z['ctrl']);rows=[];pairs={}
    for j in range(16):
        for change in (-2,2):
            goal=previous.copy();goal[j]+=change;d=restore(m,z);check=Guard(m);stop=None;phase=None
            if np.any(goal<g.lo) or np.any(goal>g.hi):stop=['native_goal_range']
            else:
                d.ctrl[:]=ticks_to_sim(goal)
                for step in range(150):
                    mujoco.mj_step(m,d);stop=check.check(d)
                    if stop:
                        for name,value in check.last['self_pairs_mm'].items():
                            pairs[name]=max(pairs.get(name,0.),float(value))
                        break
            w=contact_wrench(m,d,g.hand)
            rows.append(dict(motor_id=j+1,goal_change_ticks=change,stop=stop,
                self_pairs_mm=check.last.get('self_pairs_mm',{}),
                gaps_mm={name:geometry.gap(d,m.geom(name).id)*1000 for name in ('rf_tip','th_tip')},
                forces_N_model_only={name:r['normal_N'] for name,r in w['by_geom'].items()}))
    sid=np.array(SIM_TO_MOTOR_IDS)-1;states=[]
    for sim,mid in enumerate(sid):
        states.append(dict(motor_id=int(mid+1),goal_ticks=int(previous[mid]),lower=float(g.lo[mid]),upper=float(g.hi[mid]),
            measured_radians=float(z['qpos'][sim]),goal_radians=float(z['ctrl'][sim]),
            native_lower_radians=float(g.qlo[sim]),native_upper_radians=float(g.qhi[sim])))
    report=dict(status='local_bounded_closing_diagnostics',source_sha256=sha(source),source=str(source.relative_to(OUT)),
        model_provenance=provenance,joint_states=states,probes=rows,blocked_self_pairs_maximum_mm=pairs,
        actual_persistent_state_not_changed=True,probes_are_separate_model_branches=True,
        physical_actions=0,devices_opened=0,hardware_execution_allowed=False)
    save_json(OUT/'next_cycle/handoff_refinement_20261110/local_move_diagnostics.json',report)
    print(json.dumps(dict(blocked_pairs=pairs,safe_probes=[r for r in rows if not r['stop']]),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
