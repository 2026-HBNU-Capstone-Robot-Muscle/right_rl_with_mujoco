"""Fit archived fingertip geometry at a reached cube pose in IK copies only.

Nothing here moves a physical cube or certifies a dynamic approach. The output
separates static contact availability from the still-required free-cube replay.
"""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle';sys.path.insert(0,str(WORK))
import mujoco
from scipy.optimize import least_squares
from native_cycle import assets,restore,Guard,motor,sha,save_json
PARENT=ROOT/'right_hand_rotation_20261112/support_first_transfer'
OUT=PARENT/'reached_three_finger_geometry'
REF=WORK/'next_cycle/handoff_refinement_20261110/ring_thumb_MF_support_feedback_v2/force0.5/end_state.npz'

def main():
    OUT.mkdir(exist_ok=False);m,initial,_,_,provenance=assets('active_ccd50');g=Guard(m);cube=m.geom('cube').id
    ref=restore(m,dict(np.load(REF,allow_pickle=False)))
    Cr=ref.geom_xpos[cube].copy();Br=ref.geom_xmat[cube].reshape(3,3).copy()
    fingers={'th':np.arange(12,16),'mf':np.arange(4,8),'rf':np.arange(8,12)}
    local={f:Br.T@(ref.geom_xpos[m.geom(f+'_tip').id]-Cr) for f in fingers}
    results=[]
    for tag in ('start','end'):
        source=PARENT/f'lower_pad_lift_after_clear_v2/{tag}_state.npz';saved=dict(np.load(source,allow_pickle=False));d=restore(m,saved)
        C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();lag=saved['ctrl']-saved['qpos'][:16]
        fits={};rows=[]
        for f,active in fingers.items():
            tip=m.geom(f+'_tip').id;target=C+B@local[f];copy=restore(m,saved)
            # Constrain both static actual joint angles and corresponding goals.
            sid=np.asarray((8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1))-1;T=2*np.pi/4096
            lo=np.maximum(g.qlo,(np.ceil(g.lo[sid])+1-2048)*T-lag)
            hi=np.minimum(g.qhi,(np.floor(g.hi[sid])-1-2048)*T-lag)
            def residual(x):
                copy.qpos[:]=saved['qpos'];copy.qpos[active]=x;copy.ctrl[:]=copy.qpos[:16]+lag;mujoco.mj_forward(m,copy)
                return np.r_[(copy.geom_xpos[tip]-target)*1000,.03*(x-ref.qpos[active])]
            rng=np.random.default_rng(1200+int(active[0]));seeds=[d.qpos[active],ref.qpos[active]]+[rng.uniform(lo[active],hi[active]) for _ in range(6)]
            best=None
            for seed in seeds:
                fit=least_squares(residual,np.clip(seed,lo[active]+1e-8,hi[active]-1e-8),bounds=(lo[active],hi[active]),max_nfev=120,ftol=1e-6,xtol=1e-6,gtol=1e-6)
                score=np.linalg.norm(residual(fit.x)[:3])
                if best is None or score<best[0]:best=(score,fit.x.copy())
            residual(best[1]);fits[f]=best[1]
            contact=[c for c in copy.contact if {int(c.geom1),int(c.geom2)}=={cube,tip}]
            support,own=g.contacts(copy)
            row=dict(finger=f,centroid_fit_error_mm=float(best[0]),target_cube_frame_mm=(local[f]*1000).tolist(),
                     cube_tip_contact=bool(contact),minimum_tip_gap_mm=min((float(c.dist)*1000 for c in contact),default=None),
                     self_pairs_mm=own,support_pairs_mm=support,goal_ticks=motor(copy.ctrl).tolist())
            rows.append(row);print(json.dumps(dict(source=tag,**row)),flush=True)
        combined=restore(m,saved)
        for f,active in fingers.items():combined.qpos[active]=fits[f]
        combined.ctrl[:]=combined.qpos[:16]+lag;mujoco.mj_forward(m,combined)
        support,own=g.contacts(combined);contacts={f:[] for f in fingers}
        for c in combined.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            other=b if a==cube else a;n=m.geom(other).name or ''
            for f in fingers:
                if n.startswith(f+'_'):contacts[f].append(dict(geom=n,gap_mm=float(c.dist)*1000))
        folder=OUT/tag;folder.mkdir();np.savez_compressed(folder/'static_ik.npz',qpos=combined.qpos,ctrl=combined.ctrl)
        r=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),fingers=rows,
               combined_contacts=contacts,self_pairs_mm=own,support_pairs_mm=support,
               goal_ticks=motor(combined.ctrl).tolist(),cube_fixed_in_ik_only=True,dynamic_approach_certified=False)
        save_json(folder/'review.json',r);results.append(r)
    save_json(OUT/'review.json',dict(rows=results,model_provenance=provenance,reference_sha256=sha(REF),
             devices_opened=0,hardware_execution_allowed=False,native_ranges_unchanged=True))

if __name__=='__main__':main()
