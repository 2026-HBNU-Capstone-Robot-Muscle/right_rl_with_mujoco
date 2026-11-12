"""Replay archived actual encoders, exposing prediction/observation mismatch.

No motor interface is imported. Historical placement is uncertain and is
deliberately varied; results cannot certify a physical ninety degree rotation.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import hashlib
import json
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from replay_original_sequence_20261024 import setup
from review_original_sequence_20261024 import ROOT, OUT, save
from hand_motor_mapping import ticks_to_sim


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--same-session',action='store_true');args=parser.parse_args()
    m,manifest=setup();cube=m.geom('cube').id;body=int(m.geom_bodyid[cube])
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    old=json.loads((OUT/'ninety_degree_prefix_review.json').read_bytes())['candidates'][0]
    p=old['placement'];forward=np.array([1.,0,0]);forward-=up*(up@forward);forward/=np.linalg.norm(forward)
    lateral=np.cross(up,forward);results=[];sources=[]
    registered=json.loads((OUT/'same_session_marker_registration.json').read_bytes()) if args.same_session else None
    for filename in ('leap_sequence_flip_20261011_051350_713486.json',
                     'leap_sequence_flip_20261011_051518_433595.json'):
        blob=(ROOT/filename).read_bytes();trial=json.loads(blob)
        placed=[s for s in trial['samples'] if s['phase']=='place_cube'][-1]
        rows=[placed]+[s for s in trial['samples'] if s['phase'] in {f'stage_{i}' for i in range(1,7)}]
        times=np.array([s['t'] for s in rows]);times-=times[0]
        trace=ticks_to_sim([s['actual'] for s in rows])
        if registered:
            folder='sequence_vision_20261011_051349_782086' if '051350' in filename else 'sequence_vision_20261011_051517_532925'
            registration=next(v for v in registered['folders'] if v['folder']==folder)
            placement=next(v for v in registration['registered'] if v['image']=='placement_at_space.png')
            p=dict(center_m=placement['center_world_m'],
                quat_wxyz=Rotation.from_matrix(placement['rotation_world']).as_quat()[[3,0,1,2]].tolist(),
                source_image=folder+'/placement_at_space.png',single_marker_pose_provisional=True)
        sources.append(dict(file=filename,sha256=hashlib.sha256(blob).hexdigest(),samples=len(rows),duration_s=float(times[-1])))
        for friction in (.4,.8,1.2):
            for offset in (np.zeros(3),forward*.003,-forward*.003,lateral*.003,-lateral*.003):
                d=mujoco.MjData(m);d.qpos[:16]=trace[0];d.ctrl[:]=trace[0]
                center=np.array(p['center_m'])+offset;d.qpos[-7:]=np.r_[center,p['quat_wxyz']]
                m.geom_friction[cube,0]=friction;mujoco.mj_forward(m,d)
                initial_overlap=max([-c.dist*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
                def step(q,velocity):
                    d.qpos[:16]=q;d.qvel[:16]=velocity;d.ctrl[:]=q;mujoco.mj_step(m,d)
                for _ in range(round(1.5/m.opt.timestep)):step(trace[0],np.zeros(16))
                R0=d.xmat[body].reshape(3,3).copy();p0=d.qpos[-7:-4].copy()
                face_initial=int(np.argmax(np.r_[R0.T@up,-R0.T@up]))
                elapsed=0.;snapshots=[];last_phase='place_cube'
                while elapsed<=times[-1]+1.:
                    q=np.array([np.interp(elapsed,times,trace[:,j]) for j in range(16)])
                    nxt=np.array([np.interp(elapsed+m.opt.timestep,times,trace[:,j]) for j in range(16)])
                    step(q,(nxt-q)/m.opt.timestep);elapsed+=m.opt.timestep
                    index=min(np.searchsorted(times,elapsed,side='right')-1,len(rows)-1)
                    phase=rows[max(0,index)]['phase']
                    if phase!=last_phase or elapsed>=times[-1]+1.:
                        R=d.xmat[body].reshape(3,3).copy();rv=Rotation.from_matrix(R@R0.T).as_rotvec()
                        touching=[]
                        for c in d.contact:
                            a,b=int(c.geom1),int(c.geom2)
                            if cube in (a,b) and c.dist<.0003:
                                other=b if a==cube else a;name=m.body(int(m.geom_bodyid[other])).name
                                if name=='palm' or name.startswith(('th_','if_','mf_','rf_')):touching.append(name)
                        snapshots.append(dict(preceding_phase=last_phase,elapsed_s=float(elapsed),
                            angle_change_deg=float(np.degrees(np.linalg.norm(rv))),
                            up_face_changed=int(np.argmax(np.r_[R.T@up,-R.T@up]))!=face_initial,
                            displacement_mm=float(np.linalg.norm(d.qpos[-7:-4]-p0)*1000),
                            hand_contact_bodies=sorted(set(touching))))
                        last_phase=phase
                results.append(dict(source=filename,friction=friction,offset_m=offset.tolist(),
                    placement=p,initial_overlap_mm=initial_overlap,snapshots=snapshots))
            print('MEASURED_TRACE',filename,'friction',friction,'runs',len(results),flush=True)
    valid=[r for r in results if r['initial_overlap_mm']<=.5]
    summary=dict(runs=len(results),nonoverlapping_initial_cases=len(valid),
        maximum_initial_overlap_mm=max(r['initial_overlap_mm'] for r in results),
        nonoverlapping_cases_with_predicted_up_face_change_before_final=sum(
            any(s['up_face_changed'] for s in r['snapshots'][:-1]) for r in valid),
        historical_user_report_green_stayed_up=True,
        simulation_observation_disagreement=(any(any(s['up_face_changed'] for s in r['snapshots'][:-1]) for r in valid)
            if valid else None),
        no_evaluable_initial_case=not bool(valid),
        preserve_this_disagreement_do_not_label_model_calibrated=True)
    filename='same_session_measured_trace_replay.json' if args.same_session else 'measured_trace_replay.json'
    save(OUT/filename,dict(sources=sources,summary=summary,runs=results,
        placement=p,support=manifest,prescribed_measured_hand_angles=True,
        placement_and_friction_not_calibrated=True,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False))
    print('SUMMARY',json.dumps(summary),flush=True)


if __name__=='__main__':main()
