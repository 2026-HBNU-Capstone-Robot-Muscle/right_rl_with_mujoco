"""Independent constant-command replay of the two declared initialized holds."""
from pathlib import Path
import json
import numpy as np
import mujoco
from native_cycle import OUT,assets,restore,Guard,BOUND,ticks_to_sim,motor,save_json,sha
from whole_hand_support import sample_model
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
from upper_layer_separation_certificate_20261105 import SweepCertificate


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';geometry=base/'opposing_workspace_Z_seed122';folder=base/'opposing_grip_review_20261110';folder.mkdir(exist_ok=True)
    source=geometry/'proposal.npz';rows=[]
    for study,yaw in (('free_hold_tests',None),('free_hold_tests_yaw180',180.)):
        path=geometry/study/'force0p8/review.json';declared=json.loads(path.read_text());endfile=path.parent/'end_state.npz';saved=dict(np.load(endfile,allow_pickle=False))
        model,initial,_,_,provenance=assets('active_ccd50');cube=model.geom('cube').id;z=dict(np.load(source,allow_pickle=False))
        if yaw is not None:
            target=upright_yaw_target(initial['initial_cube_rotation'],initial['up'],yaw);q=np.empty(4);mujoco.mju_mat2Quat(q,target.ravel());z['qpos']=z['qpos'].copy();z['qpos'][-4:]=q
        d=restore(model,z);d.qvel[:]=0;d.qacc_warmstart[:]=0;d.ctrl[:]=ticks_to_sim(np.rint(motor(saved['ctrl'])).astype(int));mujoco.mj_forward(model,d)
        guard=Guard(model);assert not guard.check(d);C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();cert=SweepCertificate(model,guard.hand,.3)
        count=supported=certified=mf=th=0;lower=float('inf');shift=angle=0.
        for index in range(7050):
            mujoco.mj_step(model,d);assert not guard.check(d),guard.last
            assert np.linalg.norm(d.qpos[-7:-4]-initial['initial_cube_center'])<=.08
            if index>=3000:
                count+=1;load=sample_model(model,d,guard.hand,cube);supported+=load['supported_sample'];value,bad=cert.query(d,d.geom_xpos[cube],d.geom_xmat[cube].reshape(3,3));lower=min(lower,value);certified+=not bad and value>.3
                forces={name:sum(v['normal_N'] for g,v in load['by_geom'].items() if (model.body(int(model.geom_bodyid[model.geom(g).id])).name or '').startswith(name+'_')) for name in ('mf','th')}
                mf+=forces['mf']>.1;th+=forces['th']>.1;shift=max(shift,float(np.linalg.norm(d.geom_xpos[cube]-C)*1000));angle=max(angle,orientation_error_deg(d.geom_xmat[cube].reshape(3,3),B))
        errors={key:float(np.max(abs(getattr(d,key)-saved[key]))) if saved[key].size else 0. for key in BOUND}
        assert max(errors.values())==0.,errors
        assert count==certified==supported==mf==th==4050
        assert abs(lower-declared['minimum_upper_lower_mm'])<1e-10
        assert abs(shift-declared['maximum_station_drift_mm'])<1e-10 and abs(angle-declared['maximum_orientation_drift_deg'])<1e-9
        rows.append(dict(study=study,declared_review=path.relative_to(OUT).as_posix(),declared_review_sha256=sha(path),source_proposal_sha256=sha(source),model_provenance=provenance,
            final_bound_state_errors=errors,hold_samples=count,upper_certified_samples=certified,whole_hand_supported_samples=supported,MF_loaded_samples=mf,TH_loaded_samples=th,
            minimum_upper_lower_mm=lower,station_drift_mm=shift,orientation_drift_deg=angle,initialized_replay_passed=True,arrival_path_not_evaluated=True))
        print(json.dumps(dict(study=study,matched_bound_state=True,certified_samples=certified,minimum_upper_mm=lower)),flush=True)
    save_json(folder/'hold_replay_verification.json',dict(status='passed',rows=rows,initialized_holds_not_first_or_next_rotation=True,source_script_sha256=sha(Path(__file__)),
        full_two_turn_cycle_passed=False,new_SAC_training_steps=0,devices_opened=0,hardware_execution_allowed=False))


if __name__=='__main__':main()
