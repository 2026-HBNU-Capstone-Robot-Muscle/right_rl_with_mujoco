"""Make a model friction sweep apply to both contacting surfaces.

This is an offline material hypothesis, not an identified physical coefficient.
Contact geometry, solver softness, joint limits, and motor properties are retained.
"""
import numpy as np

def apply_material(model, coefficient, hand_bodies, cube_geom):
    coefficient = float(coefficient)
    if coefficient not in (.2,.3,.5,.8,1.):
        raise ValueError('Unreviewed material hypothesis')
    before = model.geom_friction.copy()
    pairs_before = model.pair_friction.copy()
    geoms = [i for i in range(model.ngeom) if int(model.geom_bodyid[i]) in hand_bodies]
    geoms.append(int(cube_geom))
    model.geom_friction[geoms,0] = coefficient
    pairs = []
    for i in range(model.npair):
        a,b = int(model.pair_geom1[i]),int(model.pair_geom2[i])
        if cube_geom in (a,b) and int(model.geom_bodyid[b if a==cube_geom else a]) in hand_bodies:
            model.pair_friction[i,:2] = coefficient
            pairs.append(i)
    # Only sliding coefficients of the named material interfaces may change.
    expected = before.copy(); expected[geoms,0] = coefficient
    expected_pairs = pairs_before.copy(); expected_pairs[pairs,:2] = coefficient
    if not np.array_equal(model.geom_friction,expected) or not np.array_equal(model.pair_friction,expected_pairs):
        raise RuntimeError('Unrequested material change')
    return dict(requested_effective_sliding_coefficient=coefficient,
        measured_physical_material=False,
        policy='both surfaces; explicit cube/hand pairs also use the same sliding coefficient',
        original_tip_sliding_coefficients={name:float(before[model.geom(name).id,0])
            for name in ('if_tip','mf_tip','rf_tip','th_tip')},
        original_cube_sliding_coefficient=float(before[cube_geom,0]),
        affected_geom_count=len(geoms),affected_explicit_pair_count=len(pairs),
        torsional_rolling_and_other_contact_parameters_preserved=True)

def cube_contact_friction(model, data, hand_bodies, cube_geom):
    contacts = []
    for contact in data.contact:
        a,b = int(contact.geom1),int(contact.geom2)
        if cube_geom not in (a,b):
            continue
        other = b if a==cube_geom else a
        if int(model.geom_bodyid[other]) in hand_bodies:
            contacts.append(dict(hand_geom=model.geom(other).name,
                sliding=list(map(float,contact.friction[:2])),
                hand_priority=int(model.geom_priority[other]),
                cube_priority=int(model.geom_priority[cube_geom])))
    return contacts
