"""Replay the original saved model/path and preserve full research checkpoints."""
from pathlib import Path
import sys,time,hashlib,json,argparse
ROOT=Path(__file__).resolve().parent
if sys.platform=='win32':sys.path.insert(0,str(ROOT/'.runtime/sim-win'))
import numpy as np
import mujoco
from research_rotation_body_hold_20261104 import inspect_for,native,angle
from replay_body_hold_from_placement_20261104 import initial
from hand_motor_mapping import ticks_to_sim
from upper_layer_cylinder_clearance_20261105 import precompute,gaps

SOURCE=ROOT/'right_hand_rotation_20261106/middle_clearance_probe'
OUT=ROOT/'right_hand_rotation_20261109/middle_probe_reconstruction'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--out');args=parser.parse_args()
    if args.out:OUT=(ROOT/args.out).resolve();OUT.relative_to(ROOT)
    began=time.perf_counter();OUT.mkdir(parents=True,exist_ok=True)
    model_path=SOURCE/'evidence/model.mjb'
    model=mujoco.MjModel.from_binary_path('original.mjb',assets={'original.mjb':model_path.read_bytes()})
    path=np.load(SOURCE/'candidate.npz',allow_pickle=False)
    up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes())['up_world'])
    seed=np.array(json.loads((ROOT/'right_hand_rotation_20261030/stable_support_research/natural_recess_hypothesis_seed.json').read_bytes())['cube_qpos'])
    state,placement=initial(model,path['start'],up,seed)
    np.savez_compressed(OUT/'initial.npz',**state)
    data=mujoco.MjData(model)
    for key in ('qpos','qvel','ctrl','act','qacc_warmstart'):getattr(data,key)[:]=state[key]
    data.time=float(state['time']);mujoco.mj_forward(model,data)
    hand,support,check=inspect_for(model);_,_,nlo,nhi=native(model);cube=model.geom('cube').id
    cache=precompute(model,hand);worst={'self_mm':0.,'support_mm':0.,'cube_mm':0.,'tracking_rad':0.,'range_rad':0.}
    reference=np.load(SOURCE/'evidence/keyframe_states.npz',allow_pickle=False)
    expected={int(sample):qpos for sample,qpos in zip(reference['goal_samples'],reference['qpos'])}
    checkpoints={};nohand=longest=0.;external=False;stop=None
    for sample,goal in enumerate(path['goals'],1):
        data.ctrl[:]=ticks_to_sim(goal)
        for _ in range(75):
            mujoco.mj_step(model,data);su,sp=check(data)
            worst['self_mm']=max(worst['self_mm'],max(sp.values(),default=0.));worst['support_mm']=max(worst['support_mm'],max(su.values(),default=0.))
            worst['tracking_rad']=max(worst['tracking_rad'],float(max(abs(data.qpos[:16]-data.ctrl))))
            worst['range_rad']=max(worst['range_rad'],float(max(np.maximum(nlo-data.qpos[:16],data.qpos[:16]-nhi))))
            inhand=False
            for contact in data.contact:
                a,b=int(contact.geom1),int(contact.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a
                if int(model.geom_bodyid[other]) in hand:
                    worst['cube_mm']=max(worst['cube_mm'],max(0.,-float(contact.dist)*1000));inhand|=contact.dist<.0004
                elif other in support and contact.dist<.0004:external=True
            nohand=0. if inhand else nohand+.001;longest=max(longest,nohand)
            if worst['self_mm']>1e-6 or worst['support_mm']>.15 or worst['cube_mm']>1 or worst['tracking_rad']>.18 or worst['range_rad']>1e-6 or external or longest>.25:
                stop='original contact/range/tracking/retention criterion';break
        if sample in expected:
            full={key:getattr(data,key).copy() for key in ('qpos','qvel','ctrl','act','qacc_warmstart')}
            full.update(time=data.time,up=up,initial_cube_rotation=state['initial_cube_rotation'],initial_cube_center=state['initial_cube_center'])
            np.savez_compressed(OUT/f'checkpoint_{sample}.npz',**full)
            C=data.geom_xpos[cube];B=data.geom_xmat[cube].reshape(3,3)
            clearance=sorted(gaps(model,data,C,B,cache,True),key=lambda row:row[1])
            contacts=[]
            for ci,contact in enumerate(data.contact):
                a,b=int(contact.geom1),int(contact.geom2)
                if cube not in (a,b):continue
                other=b if a==cube else a;force=np.zeros(6);mujoco.mj_contactForce(model,data,ci,force)
                contacts.append({'geom':model.geom(other).name,'normal_N':float(force[0]),'height_from_cube_center_mm':float(-((contact.pos-C)@B[:,1])*1000)})
            checkpoints[str(sample)]={'maximum_qpos_difference_from_original':float(max(abs(data.qpos-expected[sample]))),
                                      'upper_gaps_mm':clearance,'contacts':contacts}
        if sample%300==0 or stop:print(json.dumps({'sample':sample,'total':len(path['goals']),'worst':worst,'stop':stop}),flush=True)
        if stop:break
    passed=bool(placement['passed'] and stop is None and len(checkpoints)==3 and max(row['maximum_qpos_difference_from_original'] for row in checkpoints.values())<1e-6)
    report={'status':'saved_model_cross_platform_original_path_reconstruction','mujoco':mujoco.__version__,'numpy':np.__version__,
            'source_model_sha256':sha(model_path),'source_candidate_sha256':sha(SOURCE/'candidate.npz'),
            'source_keyframes_sha256':sha(SOURCE/'evidence/keyframe_states.npz'),'script_sha256':sha(Path(__file__)),
            'initial_placement':placement,'completed_goals':sample,'total_goals':len(path['goals']),
            'worst':worst,'stop':stop,'longest_no_hand_contact_seconds':longest,'external_cube_contact':external,
            'checkpoints':checkpoints,'same_original_path_reconstructed':passed,
            'physical_registration_verified':False,'new_hardware_path_admitted':False,'devices_opened':False,
            'elapsed_seconds':time.perf_counter()-began}
    (OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({key:value for key,value in report.items() if key!='checkpoints'}),flush=True)


if __name__=='__main__':main()
