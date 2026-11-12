"""Test the reverse teacher primitive with a free, initially green-up cube.

The forward teacher reference sometimes predicted ORANGE up, the opposite
of the requested RED up. This is an independent reverse-primitive test,
not a claim that frictional manipulation is time reversible.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/original_sequence_clearance'

def main():
    p=OUT/'CAD_adapted_forward_reference_path_v2.json';blob=p.read_bytes();plan=json.loads(blob)
    assert plan['full_static_path_clearance_satisfied'];raw=np.asarray(plan['dense_goal_ticks'])[::-1].copy();q=ticks_to_sim(raw)
    original=json.loads((OUT/'CAD_tip_contact_comparison_v2.json').read_bytes())
    ends=[r['stages'][-1]['cube_center_m'] for r in original['free_cube_dynamics'] if r['model']=='CAD_mesh_tips']
    center=np.mean(ends,axis=0);start_quat=np.asarray(original['free_cube_dynamics'][0]['placement']['quat_wxyz'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'reverse_original_CAD.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;up=-m.opt.gravity/np.linalg.norm(m.opt.gravity)
    x=np.array([1.,0,0]);x-=up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x)
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def contact():
        fingers=[];external=False
        for c in d.contact:
            if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003:
                other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                if int(m.geom_bodyid[other]) in hand:fingers.append(m.body(int(m.geom_bodyid[other])).name)
                if other in support or m.geom(other).name=='floor':external=True
        return sorted(set(fingers)),external
    rows=[];dt=m.opt.timestep
    for dx in (-6.,0.,6.):
        for dy in (-6.,0.,6.):
            for friction in (.4,.8,1.2):
                for mode in ('prescribed','native_motor'):
                    mujoco.mj_resetData(m,d);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:]=[10,10,10,1,0,0,0]
                    for _ in range(round(1./dt)):
                        if mode=='prescribed':d.qpos[:16]=q[0];d.qvel[:16]=0
                        mujoco.mj_step(m,d)
                    d.qpos[-7:]=np.r_[center+(dx*x+dy*y)/1000+up*.008,start_quat];d.qvel[-6:]=0
                    m.geom_friction[cube,0]=friction;mujoco.mj_forward(m,d)
                    overlap=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
                    if overlap>.5:
                        rows.append(dict(offset_mm=[dx,dy],friction=friction,mode=mode,valid_start=False,initial_overlap_mm=overlap));continue
                    for _ in range(round(1.5/dt)):
                        if mode=='prescribed':d.qpos[:16]=q[0];d.qvel[:16]=0
                        mujoco.mj_step(m,d)
                    initial=d.qpos[-7:].copy();fingers,external=contact();R0=Rotation.from_quat(initial[-4:][[1,2,3,0]]).as_matrix()
                    green=float(np.degrees(np.arccos(np.clip((R0@np.array([0,-1.,0]))@up,-1,1))))
                    if not fingers or external or green>20:
                        rows.append(dict(offset_mm=[dx,dy],friction=friction,mode=mode,valid_start=False,settled_green_up_error_deg=green));continue
                    previous=q[0].copy();maximum_tracking=0.;maximum_self=0.;holds=[];frames=[];maxgap=0.;gap=0.
                    for i,g in enumerate(np.r_[q[1:],np.repeat(q[-1:],80,axis=0)]):
                        for k in range(1,round(.05/dt)+1):
                            target=previous+(g-previous)*k/round(.05/dt) if mode=='prescribed' else g
                            if mode=='prescribed':d.qpos[:16]=target;d.qvel[:16]=(g-previous)/.05 if i<len(q)-1 else 0
                            d.ctrl[:]=target;mujoco.mj_step(m,d)
                            maximum_tracking=max(maximum_tracking,float(np.max(abs(d.qpos[:16]-target))))
                        previous=g;fingers,external=contact();gap=0. if fingers else gap+.05;maxgap=max(maxgap,gap)
                        for c in d.contact:
                            if c.dist<0 and int(m.geom_bodyid[c.geom1]) in hand and int(m.geom_bodyid[c.geom2]) in hand:maximum_self=max(maximum_self,-float(c.dist)*1000)
                        if i%4==0:frames.append(d.qpos.tolist())
                        if i>=len(q)-1:
                            tip,axis,err=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3])*1000)
                            holds.append(dict(target_error_deg=float(np.degrees(err)),tip_deg=float(np.degrees(tip)),
                                retained=bool(fingers and not external and shift<120),shift_mm=shift,contacts=fingers))
                    result=dict(offset_mm=[dx,dy],friction=friction,mode=mode,valid_start=True,initial_overlap_mm=overlap,
                        initial_cube_qpos=initial.tolist(),settled_green_up_error_deg=green,nominal_seconds=(len(q)-1)*.05,
                        maximum_tracking_rad=maximum_tracking,maximum_hand_self_model_overlap_mm=maximum_self,
                        maximum_no_hand_contact_seconds=maxgap,final=holds[-1],
                        final_four_second_hold_reached=all(h['retained'] and h['target_error_deg']<15 for h in holds),frames=frames)
                    rows.append(result)
                    print('REVERSE_REFERENCE',dx,dy,friction,mode,'error',round(holds[-1]['target_error_deg'],2),'retained',holds[-1]['retained'],
                        'tracking',round(maximum_tracking,3),flush=True)
    (OUT/'reverse_original_turn_evaluation.json').write_text(json.dumps(dict(runs=rows,
        source_plan_sha256=hashlib.sha256(blob).hexdigest(),new_placement_hypothesis=True,reference_start_center_m=center.tolist(),
        model=manifest,goal='red up and green toward initial orange front',
        cube_goal_never_written_to_physics=True,motor_parameters_identified=False,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False),indent=2),encoding='utf-8')

if __name__=='__main__':main()
