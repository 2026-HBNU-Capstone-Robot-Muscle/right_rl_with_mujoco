"""Observe the tiny MF/palm contact in bounded, initialized model holds.

All original guard violations remain reported. Continuing a virtual diagnostic
through one tiny pair overlap does not approve that path or change any physical
or original model limits. Contact forces and collision geometry remain active.
"""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,save_json,sha
from cube_task_frame_20261105 import orientation_error_deg
from whole_hand_support import sample_model
from convex_distance_check_20261104 import CheckedDistance


def main():
    folder=OUT/'next_cycle/handoff_refinement_20261110/palm_contact_persistence'
    if folder.exists():raise RuntimeError('Preserve previous hold diagnostic')
    model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id
    tip=model.geom('mf_tip').id;palm=model.geom('palm_collision_6').id
    checked=CheckedDistance(model,tip,[palm]) if int(model.geom_type[tip])==6 else None
    sources=[OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15/end_state.npz',
             OUT/'next_cycle/whole_hand_recovery_cp1700_w0.15_coupled/end_state.npz']
    pool_path=OUT/'next_cycle/handoff_refinement_20261110/early_CCD50_states/state_pool.npz'
    pool=np.load(pool_path,allow_pickle=False);pos=int(np.flatnonzero(pool['indices']==1700)[0])
    base=dict(initial);base.update({key:pool[key][pos] for key in BOUND});base['time']=float(pool['time'][pos])
    states=[('original_checkpoint1700',base,pool_path)]
    states.extend((path.parent.name,dict(np.load(path,allow_pickle=False)),path) for path in sources)
    rows=[];pair_name='mf_tip|palm_collision_6'
    for name,state,path in states:
        d=restore(model,state);guard=Guard(model)
        B0=d.geom_xmat[cube].reshape(3,3).copy();C0=d.geom_xpos[cube].copy()
        violated=0;first=None;stop=[];supported=0;max_angle=max_shift=0.;minimum_gap=float('inf')
        sampled=[]
        for step in range(4050):
            mujoco.mj_step(model,d);reasons=guard.check(d)
            gap=(checked.gap(d,palm) if checked is not None else mujoco.mj_geomDistance(model,d,tip,palm,.1,np.zeros(6)))*1000
            minimum_gap=min(minimum_gap,float(gap))
            own=guard.last['self_pairs_mm'];only_tiny_pair=(set(own)<={pair_name} and max(own.values(),default=0.)<=.01)
            if reasons:
                violated+=1
                if first is None:first=dict(substep=step+1,reasons=reasons,guard=guard.last.copy())
            stop=[reason for reason in reasons if reason!='self_mm' or not only_tiny_pair]
            if np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])>.08:stop.append('cube_translation_over80mm')
            support=sample_model(model,d,guard.hand,cube);supported+=support['supported_sample']
            angle=orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B0)
            shift=float(np.linalg.norm(d.geom_xpos[cube]-C0)*1000)
            max_angle=max(max_angle,angle);max_shift=max(max_shift,shift)
            if step%750==0 or stop:
                sampled.append(dict(substep=step+1,palm_gap_mm=float(gap),cube_angle_drift_deg=angle,
                                    cube_center_drift_mm=shift,upward_support_N=support['upward_support_N']))
            if stop:break
        rows.append(dict(source=name,source_path=path.relative_to(OUT).as_posix(),source_sha256=sha(path),
            simulated_substeps=step+1,seconds=(step+1)*float(model.opt.timestep),stop=stop,
            original_guard_violating_substeps=violated,first_original_guard_violation=first,
            original_strict_hold_passed=violated==0 and step+1==4050,
            maximum_cube_angle_drift_deg=max_angle,maximum_cube_center_drift_mm=max_shift,
            supported_samples=supported,supported_fraction=supported/(step+1),minimum_mf_palm_gap_mm=minimum_gap,
            worst=guard.worst,trace=sampled,
            diagnostic_continuation_pair_limit_mm=.01,
            only_observation_continues_not_a_qualified_motion=True))
        print(json.dumps({key:rows[-1][key] for key in ('source','simulated_substeps','stop','original_guard_violating_substeps',
            'maximum_cube_angle_drift_deg','maximum_cube_center_drift_mm','minimum_mf_palm_gap_mm','supported_fraction')}),flush=True)
    folder.mkdir()
    report=dict(status='bounded_model_palm_contact_hold_observation',model_provenance=provenance,
        rows=rows,mf_tip_geom_type=int(model.geom_type[tip]),independent_SAT_used=checked is not None,
        original_guard_violations_preserved=True,model_geometry_and_forces_unchanged=True,
        no_physical_clearance_or_contact_tolerance_inferred=True,
        devices_opened=0,hardware_execution_allowed=False)
    save_json(folder/'review.json',report)
    (folder/'source_probe_palm_contact_persistence.py').write_bytes(Path(__file__).read_bytes())


if __name__=='__main__':main()
