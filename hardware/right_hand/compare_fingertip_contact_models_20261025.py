"""Native box versus existing CAD fingertip contacts, offline diagnostics."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from mujoco_playground import registry
from adapt_original_sequence_clearance_20261024 import build as build_boxes,OUT
from fingertip_cad_model_20261025 import build as build_cad
from review_original_sequence_20261024 import load_source
from replay_original_sequence_20261024 import simulate
from hand_motor_mapping import hw_to_sim,ticks_to_sim


def main():
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    boxes,box_manifest=build_cad(env,OUT/'box_tip_control_scene.xml',use_cad_tips=False)
    cad,cad_manifest=build_cad(env,OUT/'cad_tip_comparison_scene.xml')
    for m in (boxes,cad):
        from sac_cube_model import set_cube_physics
        set_cube_physics(m,side_m=.055,mass_kg=.1)
    d0,d1=mujoco.MjData(boxes),mujoco.MjData(cad)
    names,hw,source=load_source();tip_comparison=[]
    for m,d in ((boxes,d0),(cad,d1)):
        d.qpos[:16]=hw_to_sim(hw[0]);d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
    for name in ('if_tip','mf_tip','rf_tip','th_tip'):
        a,b=boxes.geom(name).id,cad.geom(name).id;mesh=int(cad.geom_dataid[b])
        start=int(cad.mesh_vertadr[mesh]);count=int(cad.mesh_vertnum[mesh]);vertices=cad.mesh_vert[start:start+count]
        world=vertices@d1.geom_xmat[b].reshape(3,3).T+d1.geom_xpos[b]
        boxlocal=(world-d0.geom_xpos[a])@d0.geom_xmat[a].reshape(3,3)
        tip_comparison.append(dict(name=name,old_box_dimensions_mm=(2*boxes.geom_size[a]*1000).tolist(),
            cad_convex_hull_volume_to_box_ratio=float(ConvexHull(vertices).volume/(8*np.prod(boxes.geom_size[a]))),
            maximum_CAD_vertex_outside_old_box_mm=float(np.max(abs(boxlocal)-boxes.geom_size[a])*1000),
            CAD_mesh_vertices=count,geometry_is_not_a_uniform_shrink=True))
    def collisions(m,d,q):
        d.qpos[:16]=q;d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
        found=[]
        for c in d.contact:
            if c.dist>=-1e-7:continue
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb or a in support and hb or b in support and ha:
                found.append(dict(pair=[m.geom(a).name,m.geom(b).name],model_overlap_mm=-float(c.dist)*1000))
        return found
    audits=[]
    for label,m,d in (('native_box_tips',boxes,d0),('CAD_mesh_tips',cad,d1)):
        rows=[]
        for stage in range(1,7):
            maximum=0.;samples=0;witness=None
            for k in range(71):
                q=hw_to_sim(hw[stage-1]+(hw[stage]-hw[stage-1])*k/70)
                c=collisions(m,d,q)
                if c:
                    samples+=1;worst=max(c,key=lambda r:r['model_overlap_mm'])
                    if worst['model_overlap_mm']>maximum:maximum=worst['model_overlap_mm'];witness=dict(fraction=k/70,**worst)
            rows.append(dict(stage=stage,colliding_samples=samples,maximum_model_overlap_mm=maximum,witness=witness))
        audits.append(dict(model=label,original_sequence_segments=rows))
        print('TIP_MODEL_STATIC',label,[(r['stage'],round(r['maximum_model_overlap_mm'],3)) for r in rows],flush=True)
    previous=json.loads((OUT.parent/'original_sequence_review/visualized_replay.json').read_bytes())
    placement=previous['placement'];dynamics=[]
    for label,m in (('native_box_tips',boxes),('CAD_mesh_tips',cad)):
        for friction in (.4,.8,1.2):
            row=simulate(m,hw,placement,friction,stop_stage=2,final_hold_s=4.)
            row['model']=label;dynamics.append(row)
            final=row['stages'][-1]
            print('TIP_MODEL_DYNAMICS',label,friction,'held',row['final_retained'],
                'nearest_90_error',round(min(final['target_errors_deg'].values()),2),flush=True)
    report=dict(source=source,tip_geometry_comparison=tip_comparison,static_audits=audits,
        free_cube_dynamics=dynamics,manifests=[box_manifest,cad_manifest],
        primary_CAD_shapes_used_not_collision_tolerance_changes=True,
        hardware_execution_allowed=False,hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False,
        contact_model_validation_against_physical_forces_pending=True)
    report['matched_housing_proxy_coverage_in_both_models']=True
    report['CAD_collision_and_visual_transforms_verified']=True
    (OUT/'CAD_tip_contact_comparison_v2.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__':main()
