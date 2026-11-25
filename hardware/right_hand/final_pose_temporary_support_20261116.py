"""Explicit unmeasured footprint hypothesis for the user's temporary support.

Retains book/glass/wood thicknesses, every collision geom and contact force.
The former book model extended 1 metre sideways. Here the book footprint
covers the palm CAD plus15mm on both sides, in its own lateral axis.
This is an operator-approved approximation, not a physical measurement.
"""
import numpy as np

POLICY=dict(version='temporary_support_palm_width_plus15mm_assumption_20261116',
    user_requested_temporary_support_approximation=True,physical_dimensions_measured=False,
    palm_CAD_lateral_margin_mm=15,book_height_changed=False,glass_or_wood_thickness_changed=False,
    base_projection_from_fixed_mount_front_mm=0,former_base_projection_mm=50,
    book_front_hypothesis_offset_mm=-10,
    removed_collision_geoms=0,contact_forces_preserved=True)

def build(env,scene_path,*,book_projection_mm=-10.,**kwargs):
    import mujoco
    from fingertip_cad_model_20261025 import build as previous_build
    model,manifest=previous_build(env,scene_path,book_projection_mm=book_projection_mm-10,**kwargs)
    d=mujoco.MjData(model);mujoco.mj_forward(model,d)
    book=model.geom('support_book').id;palm=model.geom('palm_visual').id
    if int(model.geom_bodyid[book])!=0:raise RuntimeError('Support must be fixed world geometry')
    mesh=int(model.geom_dataid[palm]);start=int(model.mesh_vertadr[mesh]);count=int(model.mesh_vertnum[mesh])
    verts=model.mesh_vert[start:start+count]@d.geom_xmat[palm].reshape(3,3).T+d.geom_xpos[palm]
    axis=d.geom_xmat[book].reshape(3,3)[:,1]
    projection=(verts-d.geom_xpos[book])@axis
    lower,upper=float(projection.min())-.015,float(projection.max())+.015
    old_size=model.geom_size[book].copy();old_pos=model.geom_pos[book].copy()
    model.geom_pos[book]+=axis*(lower+upper)/2
    model.geom_size[book,1]=(upper-lower)/2
    # The temporary photo shows a support under the body, rather than the old
    #50mm forward overhang beneath the free finger links. Keep the rear edge
    #and thickness; assume the front ends at the mount. Not a measured dimension.
    for name in ('support_glass','support_wood'):
        g=model.geom(name).id
        if int(model.geom_bodyid[g])!=0:raise RuntimeError('Base must be fixed world geometry')
        forward=d.geom_xmat[g].reshape(3,3)[:,0]
        model.geom_size[g,0]-=.025
        model.geom_pos[g]-=forward*.025
    mujoco.mj_setConst(model,mujoco.MjData(model))
    manifest['temporary_support_assumption']=dict(POLICY,
        former_book_size_mm=(old_size*1000).tolist(),book_size_mm=(model.geom_size[book]*1000).tolist(),
        former_book_position_mm=(old_pos*1000).tolist(),book_position_mm=(model.geom_pos[book]*1000).tolist())
    return model,manifest
