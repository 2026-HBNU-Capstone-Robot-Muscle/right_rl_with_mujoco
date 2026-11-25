"""List the in-process simulator cube and finger geometry; opens no devices."""

import json
from pathlib import Path

import mujoco
from mujoco_playground import registry

ROOT = Path(__file__).resolve().parent


def main():
    cfg = registry.get_default_config('LeapCubeFaceUp')
    cfg.impl = 'jax'
    env = registry.load('LeapCubeFaceUp', config=cfg)
    model = env.mj_model
    rows = []
    for gid in range(model.ngeom):
        body = int(model.geom_bodyid[gid])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if body_name == 'cube' or body_name.startswith(('palm', 'th_', 'if_', 'mf_', 'rf_')):
            rows.append(dict(
                geom=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid),
                body=body_name,
                type=int(model.geom_type[gid]),
                size=model.geom_size[gid].tolist(),
                contype=int(model.geom_contype[gid]),
                conaffinity=int(model.geom_conaffinity[gid]),
                mesh_id=int(model.geom_dataid[gid]) if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH else None,
            ))
    report = dict(status='offline_geometry_inventory', motor_commands_sent=0,
                  cube_geoms=[r for r in rows if r['body'] == 'cube'],
                  finger_collision_geoms=sum(r['body'].startswith(('th_', 'if_', 'mf_', 'rf_'))
                                             and r['contype'] != 0 for r in rows),
                  palm_geoms=[r for r in rows if r['body'] == 'palm'])
    (ROOT / 'sac_cube_geom_audit_result.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
