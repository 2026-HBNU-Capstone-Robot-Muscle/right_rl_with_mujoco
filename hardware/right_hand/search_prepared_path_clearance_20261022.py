"""Offline small thumb-clearance variants. Never opens a hardware device."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'prepared_cube_review_20261022'


def main():
    review=json.loads((OUT/'placement_review.json').read_bytes())
    old=json.loads((ROOT/'full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json').read_bytes())
    start=np.array(review['placement_goal_ticks']);actual=ticks_to_sim(review['placement_position_ticks'])
    original=np.array([old['start_ticks']]+old['goals_id_order'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build_with_middle_thumb_hulls(env)
    set_cube_physics(m,side_m=.055,mass_kg=.100)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    cube=m.geom('cube').id;cmasks=(int(m.geom_contype[cube]),int(m.geom_conaffinity[cube]))
    bounds=m.jnt_range[m.actuator_trnid[:,0]].copy();control=m.actuator_ctrlrange.copy()
    reset=np.array(json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates'][0]['qpos'][-7:])
    def contact(d):
        maximum=0.;pair=None;sup=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0,-float(c.dist)*1000)
            if (a in support or b in support) and a!=cube and b!=cube:sup=max(sup,depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and depth>maximum:
                maximum=depth;pair=[m.geom(a).name,m.geom(b).name]
        return maximum,sup,pair
    def make(delta,fade):
        n=len(original);t=np.arange(n)
        initial=np.maximum(0,1-t/20)[:,None]*(start-original[0])[None,:]
        raw=original+initial
        # Full-path offset ramps in from the already reached -64 tick start.
        raw[:,0]+=np.minimum(t/20,1)*(delta if not fade else delta*np.maximum(0,1-t/(n-1)))
        return np.rint(raw).astype(int)
    variants=[]
    for delta in (0,-16,-32,-64,-96,-128,16,32,64):
        for fade in (False,True) if delta else (False,):
            raw=make(delta,fade);goals=ticks_to_sim(raw);d=mujoco.MjData(m)
            violation=bool(np.any(goals<bounds[:,0]) or np.any(goals>bounds[:,1]) or
                np.any(goals<control[:,0]) or np.any(goals>control[:,1]))
            worst=0.;pair=None;when=None;supportworst=0.
            for i,goal in enumerate(goals):
                d.qpos[:16]=goal;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
                depth,sup,p=contact(d);supportworst=max(supportworst,sup)
                if depth>worst:worst=depth;pair=p;when=i
            row=dict(delta_id1=delta,fade=fade,static_overlap_mm=worst,pair=pair,worst_sample=when,
                static_support_mm=supportworst,outside_native_range=violation)
            # Native dynamics are needed: static clearance alone can be lost
            # when the ideal actuators lag, even before a cube is added.
            if not violation:
                dynamic=[]
                for loaded in (False,True):
                    m.geom_contype[cube]=cmasks[0] if loaded else 0
                    m.geom_conaffinity[cube]=cmasks[1] if loaded else 0
                    d=mujoco.MjData(m);d.qpos[:16]=actual;d.qpos[-7:]=reset;d.ctrl[:]=goals[0];mujoco.mj_forward(m,d)
                    origin=d.qpos[-7:].copy();peak=track=sup=0.;worstpair=None
                    for goal in [goals[0]]*20+list(goals)+[goals[-1]]*20:
                        d.ctrl[:]=goal
                        for _ in range(round(.05/m.opt.timestep)):
                            mujoco.mj_step(m,d);depth,s,p=contact(d);sup=max(sup,s)
                            if depth>peak:peak=depth;worstpair=p
                        track=max(track,float(np.max(abs(d.qpos[:16]-goal))))
                    tip,axis,error=measure(origin[-4:],d.qpos[-4:])
                    dynamic.append(dict(loaded=loaded,max_self_overlap_mm=peak,pair=worstpair,max_support_mm=sup,
                        max_tracking_rad=track,tip_deg=float(np.degrees(tip)) if loaded else None,
                        off_axis_deg=float(np.degrees(axis)) if loaded else None,
                        target_error_deg=float(np.degrees(error)) if loaded else None,
                        shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-origin[:3])*1000) if loaded else None))
                row['dynamic']=dynamic
            variants.append(row);print(json.dumps(row),flush=True)
    m.geom_contype[cube],m.geom_conaffinity[cube]=cmasks
    assert np.array_equal(bounds,m.jnt_range[m.actuator_trnid[:,0]]) and np.array_equal(control,m.actuator_ctrlrange)
    doc=dict(status='offline_clearance_search',hardware_execution_allowed=False,motor_commands_sent=0,
        cube_reset_synthetic=True,model_manifest=manifest,variants=variants,
        caveat='Small anatomical thumb-tip changes, not trained SAC actions or physical clearance proof.')
    (OUT/'clearance_search.json').write_text(json.dumps(doc,indent=2))


if __name__=='__main__':main()
