"""Structural scope of the fingers lifted by root motors 8/12/16.

This classifies which INITIAL support contacts can be tested for separation.
It does not permit a new contact or a deeper contact, change any depth/time
bound, or approve motion. The native screen verifies this table against its
actual body tree before using it.
"""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SCOPE_PATH=ROOT/'book_support_escape_scope_20261024.json'
ROOT_JOINTS={8:'if_mcp',12:'mf_mcp',16:'rf_mcp'}
SUPPORT_GEOMS=('support_book','support_glass','support_wood')


def derive_scope(model):
    groups={};supports=[model.geom(name).id for name in SUPPORT_GEOMS]
    for motor,joint_name in ROOT_JOINTS.items():
        body=int(model.jnt_bodyid[model.joint(joint_name).id]);names=[]
        for geom in range(model.ngeom):
            ancestor=int(model.geom_bodyid[geom]);seen=set()
            while ancestor and ancestor not in seen and ancestor!=body:
                seen.add(ancestor);ancestor=int(model.body_parentid[ancestor])
            if ancestor==body:
                # Unnamed visual-only meshes cannot contact a support. Scope
                # only actual colliders, using MuJoCo's type/affinity masks.
                if not any((int(model.geom_contype[geom])&int(model.geom_conaffinity[s]))
                           or (int(model.geom_contype[s])&int(model.geom_conaffinity[geom])) for s in supports):
                    continue
                name=model.geom(geom).name
                if not isinstance(name,str) or not name or '|' in name:
                    raise ValueError('Clearing finger has an unnamed or invalid geometry')
                names.append(name)
        groups[str(motor)]=dict(root_joint=joint_name,root_body=model.body(body).name,
            geometry_names=sorted(names))
    scope=dict(method='native_root_descendants_with_support_collision_masks',root_groups=groups,
        support_geometry_names=list(SUPPORT_GEOMS),physical_calibration_verified=False)
    validate_scope(scope)
    return scope


def validate_scope(scope):
    if (scope.get('method')!='native_root_descendants_with_support_collision_masks'
            or scope.get('support_geometry_names')!=list(SUPPORT_GEOMS)
            or set(scope.get('root_groups',{}))!={str(k) for k in ROOT_JOINTS}):
        raise ValueError('Wrong support separation scope')
    used=set()
    for motor,joint in ROOT_JOINTS.items():
        row=scope['root_groups'][str(motor)];names=row.get('geometry_names')
        if (row.get('root_joint')!=joint or not isinstance(row.get('root_body'),str)
                or not isinstance(names,list) or not names or len(set(names))!=len(names)):
            raise ValueError('Invalid clearing root/body geometry group')
        for name in names:
            if not isinstance(name,str) or not name or '|' in name or name in used or name in SUPPORT_GEOMS:
                raise ValueError('Invalid or multiply classified finger geometry')
            used.add(name)
    return scope


def load_scope():return validate_scope(json.loads(SCOPE_PATH.read_bytes()))


def allowed_initial_pairs(scope=None):
    scope=load_scope() if scope is None else validate_scope(scope)
    return frozenset('|'.join(sorted((name,support)))
        for row in scope['root_groups'].values() for name in row['geometry_names'] for support in SUPPORT_GEOMS)
