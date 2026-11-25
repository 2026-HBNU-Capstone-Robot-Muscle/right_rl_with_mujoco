"""Inspect which finger motion blocks a proposed third-support handoff."""
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,save_json


def main():
    m,_,_,_,_=assets('active');z=dict(np.load(OUT/'connected_sac_and_teacher/first_quarter_end_state.npz'))
    guard=Guard(m);rows=[];base=OUT/'next_cycle/clearance_rf'
    for path in sorted(base.glob('proposal*.npz')):
        pose=np.load(path)['qpos'][:16];phases={}
        for label,active in (('middle_only',np.arange(4,8)),('middle_and_index',np.arange(1,8)),
                             ('middle_and_ring',np.arange(4,12)),('all_three',np.arange(1,12))):
            d=restore(m,z);d.qpos[active]=pose[active];mujoco.mj_forward(m,d)
            support,own=guard.contacts(d)
            phases[label]=dict(self_pairs_mm=own,support_pairs_mm=support,
                               self_mm=max(own.values(),default=0.))
        rows.append(dict(proposal=path.name,phases=phases))
    save_json(OUT/'next_cycle/contact_mode_stages.json',dict(rows=rows,devices_opened=0))
    print(json.dumps(rows),flush=True)


if __name__=='__main__':main()
