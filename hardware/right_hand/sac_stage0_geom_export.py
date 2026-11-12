"""Offline export of proximal collision boxes at archived measured stage-0 pose."""

import json
import sys
from pathlib import Path

import jax
import mujoco
import numpy as np
from mujoco_playground import registry


ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / "work" / "Leap_Test" / "deploy"))
from leap_hw import hw_to_sim


def main():
    cfg = registry.get_default_config("LeapCubeFaceUp")
    cfg.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=cfg)
    model = env.mj_model
    ticks = json.loads((ROOT / "sac_stage0_alignment_audit_result.json").read_text(encoding="utf-8"))["loaded_actual_ticks"]
    q = np.asarray(env.reset(jax.random.PRNGKey(0)).data.qpos, dtype=float)
    q[:16] = hw_to_sim(np.asarray(ticks) * 2 * np.pi / 4096)
    data = mujoco.MjData(model)
    data.qpos[:] = q
    data.ctrl[:] = q[:16]
    mujoco.mj_forward(model, data)
    geoms = {}
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if not name or "collision" not in name or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        if not name.startswith(("palm_", "if_", "mf_", "rf_", "th_")):
            continue
        center = data.geom_xpos[geom_id]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        half = model.geom_size[geom_id]
        corners = [center + rotation @ (half * np.asarray([sx, sy, sz]))
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        geoms[name] = {"center_world_m": center.tolist(),
                       "corners_world_m": [x.tolist() for x in corners]}
    result = {"source_motor_ticks": ticks, "geoms": geoms,
              "hardware_authorized": False, "note": "Model boxes only, not a validated physical overlay."}
    (ROOT / "sac_stage0_geom_export_result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Exported {len(geoms)} collision boxes", flush=True)


if __name__ == "__main__":
    main()
