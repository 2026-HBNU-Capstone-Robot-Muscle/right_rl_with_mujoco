"""Read-only geometry diagnostics for the supported RF grip; no device access."""
import json
import numpy as np
from native_cycle import OUT, assets, restore, Guard, motor
from upper_layer_cylinder_clearance_20261105 import precompute, gaps


def main():
    model, _, _, _, _ = assets('active_ccd50')
    source = OUT / 'next_cycle/handoff_refinement_20261110/RF_endpoint_CCD50_load_screen/qualified0_end_state.npz'
    data = restore(model, dict(np.load(source, allow_pickle=False)))
    guard = Guard(model); cube = model.geom('cube').id
    center = data.geom_xpos[cube].copy(); rotation = data.geom_xmat[cube].reshape(3, 3).copy()
    report = dict(source_ticks=motor(data.ctrl).tolist(), sim_motor_ids=list(__import__('native_cycle').SIM_TO_MOTOR_IDS),
        joint_degrees=(data.qpos[:16]*180/np.pi).tolist(),
        joint_range_degrees=(np.c_[guard.qlo,guard.qhi]*180/np.pi).tolist(),
        contacts=[dict(geom=model.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name,
            cube_local_mm=((c.pos-center)@rotation*1000).tolist(),distance_mm=float(c.dist*1000))
            for c in data.contact if cube in (int(c.geom1),int(c.geom2))],
        nearest_upper=[v for v in gaps(model,data,center,rotation,precompute(model,guard.hand),True) if v[1]<2])
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__ == '__main__':main()
