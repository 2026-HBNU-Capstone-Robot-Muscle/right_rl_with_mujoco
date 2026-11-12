"""Conditional small index/middle closure search; simulation only, no export."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from sac_cube_model import set_cube_physics

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_grasp_trial_20261023_044607_477240'

def main():
    blob=(FOLDER/'direct_reference_cube_pose.json').read_bytes();ref=json.loads(blob)
    h=min(ref['hypotheses'],key=lambda h:h['marker_error_px'])
    actual=np.array(ref['measured_motor_ticks'],dtype=float)
    goal=np.array(ref['goal_ticks'],dtype=int)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);set_cube_physics(m,side_m=.055,mass_kg=.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id
    center=np.array(h['cube_center_world_m']);R=np.array(h['cube_rotation_world'])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    ids=np.array([5,7,8,9,11,12]);indices=ids-1
    bound=np.array([48,64,24,48,64,24])
    native=m.jnt_range[m.actuator_trnid[:,0]].copy();ctrl=m.actuator_ctrlrange.copy()
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_glass','support_wood')}
    tips=[m.geom(n).id for n in ('if_tip','mf_tip','th_tip')]

    def state(delta,position=center):
        raw=actual.copy();raw[indices]+=delta
        q=ticks_to_sim(raw)
        # Production endpoints are rounded to integer ticks before assessment.
        d.qpos[:16]=q;d.qpos[-7:]=np.r_[position,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
        gaps=np.array([mujoco.mj_geomDistance(m,d,cube,g,1,None)*1000 for g in tips])
        peaks=np.zeros(3)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000)
            if cube in (a,b):peaks[2]=max(peaks[2],depth)
            elif a in supports or b in supports:peaks[1]=max(peaks[1],depth)
            elif int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:peaks[0]=max(peaks[0],depth)
        return gaps,peaks,q

    # Local finite differences in integer ticks; roots/PIP/DIP only.
    baseline,initial_peak,_=state(np.zeros(6,dtype=int))
    derivatives=[]
    for k,ident in enumerate(ids):
        delta=np.zeros(6,dtype=int);delta[k]=8
        gaps,_,_=state(delta)
        derivatives.append(dict(motor_id=int(ident),tip_gap_delta_per_8ticks_mm=(gaps-baseline).tolist()))
    print('BASE',baseline.tolist(),'DERIVATIVES',derivatives,flush=True)

    # Smooth FK is used by the optimizer only; assess rounded targets below.
    from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
    order=np.array(SIM_TO_MOTOR_IDS)-1
    def smooth(delta):
        raw=actual.copy();raw[indices]+=delta
        d.qpos[:16]=raw[order]*TICK-np.pi
        d.qpos[-7:]=np.r_[center,quat];d.qvel[:]=0;mujoco.mj_forward(m,d)
        distances=np.array([mujoco.mj_geomDistance(m,d,cube,g,1,None)*1000 for g in tips[:2]])
        collisions=np.zeros(m.ngeom)
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube in (a,b):continue
            bad=(a in supports or b in supports or (int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand))
            if bad:
                pen=max(0.,-float(c.dist)*1000);collisions[a]=max(collisions[a],pen);collisions[b]=max(collisions[b],pen)
        return np.r_[(distances-.25)*2,collisions*20,delta/bound*.08]
    rng=np.random.default_rng(20261023);candidates=[]
    for i in range(8):
        start=np.zeros(6) if i==0 else rng.uniform(-.4,.4,6)*bound
        fit=least_squares(smooth,start,bounds=(-bound,bound),diff_step=1e-3,max_nfev=100)
        delta=np.rint(fit.x).astype(int)
        if any(np.array_equal(delta,c['delta_vector']) for c in candidates):continue
        gaps,peaks,q=state(delta)
        hand_peak=support_peak=0.;native_ok=True
        for u in np.linspace(0,1,41):
            step=np.rint(delta*u).astype(int);_,pen,q=state(step,position=np.array([0.,0.,2.]))
            hand_peak=max(hand_peak,pen[0]);support_peak=max(support_peak,pen[1])
            native_ok &= bool(np.all(q>=native[:,0]) and np.all(q<=native[:,1]) and np.all(q>=ctrl[:,0]) and np.all(q<=ctrl[:,1]))
        cases=[]
        # Image/model uncertainty probe, not a measured confidence interval.
        for xyz in itertools.product((-3.,0.,3.),repeat=3):
            position=center+R@np.array(xyz)/1000
            g0,p0,_=state(np.zeros(6,dtype=int),position)
            g1,p1,_=state(delta,position)
            cases.append(dict(offset_cube_axes_mm=list(xyz),before_tip_gaps_mm=g0.tolist(),
                after_tip_gaps_mm=g1.tolist(),before_max_cube_overlap_mm=p0[2],after_max_cube_overlap_mm=p1[2]))
        row=dict(delta_vector=delta.tolist(),delta_by_motor_id={str(i):int(v) for i,v in zip(ids,delta)},
            max_joint_delta_degrees=float(max(abs(delta))*360/4096),
            nominal_tip_gaps_mm=gaps.tolist(),nominal_max_cube_overlap_mm=peaks[2],
            maximum_self_overlap_empty_mm=hand_peak,maximum_support_overlap_empty_mm=support_peak,
            within_native_bounds=native_ok,uncertainty_cases=cases,
            initial_low_overlap_cases=int(sum(c['before_max_cube_overlap_mm']<=1.5 for c in cases)),
            final_low_overlap_cases=int(sum(c['after_max_cube_overlap_mm']<=1.5 for c in cases)),
            opposed_near_tip_cases=int(sum(c['after_max_cube_overlap_mm']<=1.5 and all(-1.5<=x<=.8 for x in c['after_tip_gaps_mm']) for c in cases)))
        candidates.append(row)
        print('CANDIDATE',{k:v for k,v in row.items() if k!='uncertainty_cases'},flush=True)
    report=dict(status='conditional_small_closure_search',source=FOLDER.name,reference_sha256=hashlib.sha256(blob).hexdigest(),
        baseline_tip_gaps_mm=baseline.tolist(),baseline_max_cube_overlap_mm=float(initial_peak[2]),
        derivatives=derivatives,candidates=candidates,model_manifest=manifest,
        thumb_ring_goals_changed=False,native_ranges_unchanged=bool(np.array_equal(native,m.jnt_range[m.actuator_trnid[:,0]]) and np.array_equal(ctrl,m.actuator_ctrlrange)),
        registration_verified=False,hardware_execution_allowed=False,motor_commands_sent=0,
        conclusion='Exploratory static FK only. No loaded trajectory or controller is exported; the provisional manual image mount remains unverified.')
    (FOLDER/'conditional_new_closure_search.json').write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
