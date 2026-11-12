"""Offline initial grasp geometry near a palm/root tilted cube; no robot I/O."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'


def main():
    obs=json.loads((OUT/'placement_review.json').read_text(encoding='utf-8'))
    support=json.loads((OUT/'support_settling_scan.json').read_bytes())
    reset=support['dynamic_holds'][0]['qpos']
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.100)
    d=mujoco.MjData(m);base=ticks_to_sim(obs['placement_goal_ticks'])
    d.qpos[:16]=base;d.qpos[-7:]=reset[-7:];mujoco.mj_forward(m,d)
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_glass','support_wood')};cube=m.geom('cube').id
    cubeR=d.geom_xmat[cube].reshape(3,3).copy();center=d.geom_xpos[cube].copy()
    tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]
    # Index reaches the upper far edge; middle approaches the adjacent side,
    # thumb the near face. The previous two-far-face target was unreachable
    # under the support constraint and is retained in grasp_geometry_search.
    point_local=[np.array([.0275,-.020,-.013]),np.array([.012,-.010,.0275]),np.array([-.0275,-.004,0.])]
    normals=[cubeR[:,0],cubeR[:,2],-cubeR[:,0]]
    variables=np.array([0,1,2,3,4,5,6,7,12,13,14,15])
    jl=m.jnt_range[m.actuator_trnid[:,0]];cl=m.actuator_ctrlrange
    low=np.maximum(jl[:,0],cl[:,0])[variables]
    high=np.minimum(jl[:,1],cl[:,1])[variables]
    # Roots cannot retreat into the measured external support. The native
    # finger flexion range is retained, rather than a too-small +/-1rad IK box.
    for vi,qi in enumerate(variables):
        if qi in (0,4):low[vi]=max(low[vi],.65)
    target_rows=[]
    def state(x):
        q=base.copy();q[variables]=x;d.qpos[:16]=q;d.qpos[-7:]=reset[-7:];d.qvel[:]=0;mujoco.mj_forward(m,d);return q
    def residual(x):
        q=state(x);res=[]
        for geom,pl,n in zip(tips,point_local,normals):
            Rt=d.geom_xmat[geom].reshape(3,3)
            radius=float(np.abs(Rt.T@n)@m.geom_size[geom])
            target=center+cubeR@pl+n*(radius+.0005)
            res.extend((d.geom_xpos[geom]-target)*70)
        depth=np.zeros(m.ngeom)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);pen=max(0,-float(c.dist))
            hh=int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand
            hs=(a in supports and int(m.geom_bodyid[b]) in hand) or (b in supports and int(m.geom_bodyid[a]) in hand)
            if hh or hs or cube in (a,b):
                depth[a]=max(depth[a],pen);depth[b]=max(depth[b],pen)
        return np.r_[res,depth*800,(x-base[variables])*.04]
    rng=np.random.default_rng(20261022)
    for trial in range(8):
        initial=base.copy()
        if trial:
            initial[[0,4]]+=.5;initial[[2,6]]+=.25;initial[[3,7]]-=.4;initial[13]-=.6
            initial[variables]+=rng.normal(0,.12,len(variables))
        x0=np.minimum(np.maximum(initial[variables],low+1e-7),high-1e-7)
        result=least_squares(residual,x0,bounds=(low,high),diff_step=1e-4,max_nfev=200,
            xtol=1e-8,ftol=1e-8,gtol=1e-8)
        q=state(result.x);self_pen=support_pen=cube_pen=0.;pairs=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);pen=max(0,-float(c.dist)*1000)
            if cube in (a,b):cube_pen=max(cube_pen,pen)
            elif a in supports or b in supports:support_pen=max(support_pen,pen)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:self_pen=max(self_pen,pen)
            if pen>.01:pairs.append([m.geom(a).name,m.geom(b).name,pen])
        errors=[]
        for geom,pl,n in zip(tips,point_local,normals):
            radius=float(np.abs(d.geom_xmat[geom].reshape(3,3).T@n)@m.geom_size[geom])
            errors.append(float(np.linalg.norm(d.geom_xpos[geom]-(center+cubeR@pl+n*(radius+.0005)))*1000))
        row=dict(trial=trial,optimizer_success=bool(result.success),cost=float(result.cost),
            tip_target_errors_mm=errors,self_overlap_mm=self_pen,support_overlap_mm=support_pen,
            cube_overlap_mm=cube_pen,colliding_pairs=pairs,raw_ticks=sim_to_ticks(q).tolist(),
            qpos=q.tolist(),cube_qpos=reset[-7:],
            tip_centers_world_m=[d.geom_xpos[t].tolist() for t in tips])
        target_rows.append(row);print(json.dumps({k:v for k,v in row.items() if k not in ('qpos','cube_qpos')}),flush=True)
    report=dict(status='offline_tilted_initial_multiface_grasp_geometry_search',model_manifest=manifest,
        fingertip_face_target_points_cube_m=[p.tolist() for p in point_local],
        source='support_settling_scan.json dynamic_holds[0]',registration_verified=False,
        hardware_execution_allowed=False,motor_commands_sent=0,live_sac=False,rows=target_rows,
        caveat='Approximate point-contact IK only. Neither a collision-free approach nor calibrated real cube pose.')
    (OUT/'grasp_geometry_multiface_search.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':main()
