"""Read saved model contact geometry and forces, without device access."""
import json
import numpy as np
import mujoco
from native_cycle import OUT, assets, restore, Guard
from whole_hand_support import sample_model


def main():
    base = OUT / 'next_cycle/handoff_refinement_20261110'
    model, initial, _, _, _ = assets('active_ccd50')
    cube = model.geom('cube').id
    station = np.load(base/'reached_first90_reverse_quarter_probe/first90_space_hold_state.npz', allow_pickle=False)['qpos'][-7:-4]
    for rel in ('actual_A_next_MF_yaw_pressure/force0.5', 'actual_A_next_counter_thumb_arrival/axis0_sign+1_force0.6'):
        source = base/rel/'end_state.npz'
        if not source.exists():
            continue
        d = restore(model, dict(np.load(source, allow_pickle=False)))
        g = Guard(model)
        load = sample_model(model,d,g.hand,cube)
        print(json.dumps(dict(source=rel,center_error_vector_mm=((d.geom_xpos[cube]-station)*1000).tolist(),by_geom=load['by_geom'])),flush=True)
        for f in ('if','mf','rf','th'):
            tip = model.geom(f+'_tip').id
            points = np.zeros(6)
            gap = mujoco.mj_geomDistance(model,d,cube,tip,.2,points)
            n = points[3:]-points[:3]
            c = next((c for c in d.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}),None)
            if c is not None:
                n = c.frame[:3]*(1 if int(c.geom1)==cube else -1)
                pt = c.pos.copy()
            else:
                n = n/max(1e-12,np.linalg.norm(n))
                pt = points[:3]
            force = -n
            lever = float(np.cross(pt-d.geom_xpos[cube], force)@initial['up'])*1000
            print(json.dumps(dict(finger=f,gap_mm=float(gap*1000),force_world_per_normal_N=force.tolist(),yaw_lever_mm=lever)),flush=True)


if __name__=='__main__':
    main()
