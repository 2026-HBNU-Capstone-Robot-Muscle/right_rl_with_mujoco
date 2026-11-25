"""Separate kinematic reach from collision-free/reachable support feasibility."""
import argparse
import time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution
from native_cycle import OUT,assets,restore,Guard,save_json,sha,TICK,SIM_TO_MOTOR_IDS


def main():
    p=argparse.ArgumentParser();p.add_argument('--finger',choices=('rf','mf'),default='rf');a=p.parse_args()
    folder=OUT/'next_cycle'/('workspace_'+a.finger);folder.mkdir(parents=True,exist_ok=True)
    if (folder/'review.json').exists():raise RuntimeError('Preserve completed report')
    source=OUT/'connected_sac_and_teacher/first_quarter_end_state.npz'
    z=dict(np.load(source,allow_pickle=False));m,_,_,_,provenance=assets('active');d=restore(m,z);g=Guard(m)
    finger=np.arange(8,12) if a.finger=='rf' else np.arange(4,8);sid=np.array(SIM_TO_MOTOR_IDS)-1
    lag=d.ctrl-d.qpos[:16];qlo=np.maximum(g.qlo,(np.ceil(g.lo[sid])+2-2048)*TICK-lag)
    qhi=np.minimum(g.qhi,(np.floor(g.hi[sid])-2-2048)*TICK-lag)
    cube=m.geom('cube').id;tip=m.geom(a.finger+'_tip').id;cases=[];began=time.perf_counter()
    def evaluate(x):
        d.qpos[:]=z['qpos'];d.qpos[finger]=x;d.ctrl[:]=d.qpos[:16]+lag;mujoco.mj_forward(m,d)
        segment=np.zeros(6);distance=float(mujoco.mj_geomDistance(m,d,cube,tip,.5,segment))*1000
        return distance,segment
    for seed in range(4):
        def objective(x):
            distance,_=evaluate(x)
            return abs(distance+.02)+.0002*np.linalg.norm(x-z['qpos'][finger])
        result=differential_evolution(objective,list(zip(qlo[finger],qhi[finger])),seed=seed,
                                     maxiter=160,popsize=10,tol=1e-7,polish=True,updating='immediate')
        distance,segment=evaluate(result.x);support,own=g.contacts(d)
        row=dict(seed=seed,tip_cube_signed_distance_mm=distance,sim_angles=result.x.tolist(),
                 closest_segment_world=segment.tolist(),self_pairs_mm=own,support_pairs_mm=support,
                 pure_kinematic_contact_found=abs(distance)<.2,
                 collision_free_support_not_inferred=True,nfev=int(result.nfev))
        cases.append(row);print(__import__('json').dumps({k:row[k] for k in ('seed','tip_cube_signed_distance_mm','pure_kinematic_contact_found')}|dict(self_mm=max(own.values(),default=0.))),flush=True)
    np.savez_compressed(folder/'kinematic_seeds.npz',sim_indices=finger,sim_angles=[r['sim_angles'] for r in cases])
    save_json(folder/'review.json',dict(status='kinematic_reach_diagnostic_not_motion_or_collision_success',
        finger=a.finger,cases=cases,source=str(source.relative_to(OUT)),source_sha256=sha(source),
        model_provenance=provenance,native_limits_retained=True,
        self_collisions_measured_but_not_constrained_in_kinematic_diagnostic=True,
        cube_pose_and_other_joints_fixed_only_in_kinematic_diagnostic=True,
        devices_opened=0,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began))


if __name__=='__main__':main()
