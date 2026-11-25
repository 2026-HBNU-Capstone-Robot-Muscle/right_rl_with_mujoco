"""Offline grip-closure feasibility from the latest measured load observation.

No live devices or executable control export. Tests geometry before requesting
another physical run; a result may reject all proposals.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_supported_rotation_trial_20261023_034640_750784'


def main():
    reg=json.loads((FOLDER/'full_rim_constrained_cube_registration.json').read_bytes())
    f=reg['rows'][0];h=min(f['hypotheses'],key=lambda x:x['error_px'])
    trial=json.loads((FOLDER/'result.json').read_bytes())
    s=min(trial['samples'],key=lambda x:abs(x['t']-(f['camera_read_monotonic']-f['enabled_at'])))
    actual=np.array(s['actual']);base=np.array(s['goal'])
    R=np.array(h['cube_rotation_world']);center=np.array(h['cube_center_world_m'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    cube=m.geom('cube').id;support={m.geom(n).id for n in ('support_glass','support_wood')}
    palm=m.body('palm').id;hand={palm}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    limits=m.jnt_range[m.actuator_trnid[:,0]].copy();ctrls=m.actuator_ctrlrange.copy()
    d=mujoco.MjData(m)
    def state(q,pose):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=pose;d.qvel[:]=0;mujoco.mj_forward(m,d)
    def contacts():
        self_depth=support_depth=cube_depth=0.;touched=set()
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000)
            if cube in (a,b):cube_depth=max(cube_depth,depth);touched.add(m.geom(b if a==cube else a).name)
            elif a in support or b in support:support_depth=max(support_depth,depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:self_depth=max(self_depth,depth)
        return self_depth,support_depth,cube_depth,sorted(touched)
    state(actual,np.r_[center,quat]);palmR=d.xmat[palm].reshape(3,3).copy()
    proposals={'unchanged':{},'index_middle_small':{7:24,8:8,11:16},
               'index_middle_medium':{7:40,8:16,11:24},
               'tips_small':{5:24,7:16,9:16,11:8}}
    rows=[]
    for name,changes in proposals.items():
        delta=np.zeros(16,dtype=int)
        for ident,value in changes.items():delta[ident-1]=value
        hand_peak=support_peak=0.;control_ok=True
        for k in range(41):
            q=np.rint(actual+delta*k/40).astype(int);qsim=ticks_to_sim(q)
            control_ok &= bool(np.all(qsim>=limits[:,0]) and np.all(qsim<=limits[:,1]) and np.all(qsim>=ctrls[:,0]) and np.all(qsim<=ctrls[:,1]))
            state(q,[0,0,2,1,0,0,0]);sd,wd,_,_=contacts();hand_peak=max(hand_peak,sd);support_peak=max(support_peak,wd)
        feasible=opposed=0;initial_overlaps=[];final_overlaps=[]
        # Geometry uncertainty only; do not fit a pose to a desired grip.
        for xyz in itertools.product((-3.,0.,3.),repeat=3):
            p=center+palmR@np.array(xyz)/1000
            state(actual,np.r_[p,quat]);_,_,before,_=contacts()
            state(actual+delta,np.r_[p,quat]);_,_,after,touched=contacts()
            initial_overlaps.append(before);final_overlaps.append(after)
            if before<=1.5 and after<=1.5:
                feasible+=1
                opposed += bool('th_tip' in touched and ('if_tip' in touched or 'mf_tip' in touched))
        row=dict(name=name,delta_by_raw_motor_id=changes,max_static_self_overlap_mm=hand_peak,
            max_support_overlap_mm=support_peak,native_ranges_unchanged=True,within_native_bounds=bool(control_ok),
            pose_uncertainty_cases=27,initially_feasible_cases=sum(x<=1.5 for x in initial_overlaps),
            feasible_after_cases=feasible,opposed_tip_contact_cases=opposed,
            initial_overlap_range_mm=[min(initial_overlaps),max(initial_overlaps)],
            final_overlap_range_mm=[min(final_overlaps),max(final_overlaps)])
        rows.append(row);print(json.dumps(row),flush=True)
    assert np.array_equal(limits,m.jnt_range[m.actuator_trnid[:,0]]) and np.array_equal(ctrls,m.actuator_ctrlrange)
    out=dict(status='conditional_grip_geometry_screen',source=FOLDER.name,proposals=rows,
             model_manifest=manifest,registration_verified=False,hardware_execution_allowed=False,
             hardware_commands_sent=0,search_joint_bounds_widened=False,
             conclusion='Geometry screen alone does not authorize a loaded closure or establish an accurate cube start.')
    (FOLDER/'conditional_grip_closure_screen.json').write_text(json.dumps(out,indent=2),encoding='utf-8')


if __name__=='__main__':main()
