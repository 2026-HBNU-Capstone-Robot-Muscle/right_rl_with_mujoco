"""In-memory physical cube adjustment for offline LEAP MuJoCo diagnostics."""

import mujoco


def set_cube_physics(model, side_m=0.0545, mass_kg=0.100):
    """Adjust the cube collision box, mass, and uniform-box inertia in memory.

    The installed visual mesh remains unchanged. No installed XML is edited.
    """
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube")
    body_id = int(model.geom_bodyid[geom_id])
    original_side_m = 2 * float(model.geom_size[geom_id, 0])
    original_mass_kg = float(model.body_mass[body_id])
    original_inertia = model.body_inertia[body_id].copy()
    model.geom_size[geom_id, :3] = side_m / 2
    model.body_mass[body_id] = mass_kg
    model.body_inertia[body_id] = original_inertia * (
        mass_kg / original_mass_kg * (side_m / original_side_m) ** 2
    )
    mujoco.mj_setConst(model, mujoco.MjData(model))
    return {
        "original_side_m": original_side_m,
        "original_mass_kg": original_mass_kg,
        "side_m": side_m,
        "mass_kg": mass_kg,
        "inertia_assumption": "original cube is uniform; scale by mass ratio and squared side ratio",
        "visual_mesh_changed": False,
    }
