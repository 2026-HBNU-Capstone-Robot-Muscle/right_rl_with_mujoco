"""Conditional P4 seating from a two-face photo, with bounded settling.

The optical estimate is not certified registration. A height uncertainty study
is kept explicit; a model-feasible seed does not approve hardware motion.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1'
def main():
    OUT.mkdir(exist_ok=True);source=ROOT/'leap_sequence_flip_20261011_051518_433595.json';r=json.loads(source.read_bytes());phase=[s for s in r['samples'] if s['phase']=='stage_4'][-1];actual=np.asarray(phase['actual']);goal=np.asarray(phase['goal']);photos=json.loads((ROOT/'right_hand_rotation_20261026/original_measured_replay_v1/stage_cube_pose_review.json').read_bytes())['frames'];photo=next(p for p in photos if p['folder'].endswith('051517_532925') and p['image']=='stage_4_end.png');cp=np.asarray(photo['center_palm_mm']);Rp=np.asarray(photo['R_cube_palm'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);palm=m.body('palm').id;probe=mujoco.MjData(m);mujoco.mj_forward(m,probe);P=probe.xmat[palm].reshape(3,3);origin=probe.xpos[palm];up=-P[:,2];Rw=P@Rp;quat=Rotation.from_matrix(Rw).as_quat()[[3,0,1,2]];m.opt.gravity[:]=-9.81*up;m.geom_friction[cube,0]=.3;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        cp=sp=sup=0.;contact=[];external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,dep)
                    if c.dist<.0003:contact.append(m.geom(other).name)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return dict(cube_overlap_mm=cp,self_overlap_mm=sp,support_overlap_mm=sup,contacts=contact,external_cube_contact=external)
    rows=[];options=[]
    for lift in (0.,2.,4.,6.,8.):
        center=origin+P@cp+up*lift*.001;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=np.r_[center,quat];mujoco.mj_forward(m,d);before=observe(d);row=dict(height_uncertainty_offset_mm=lift,initial_contacts=before)
        if before['cube_overlap_mm']<=1 and before['self_overlap_mm']<=.01 and before['support_overlap_mm']<=.15:
            initial=d.qpos[-7:].copy();maxcp=before['cube_overlap_mm']
            for i in range(1000):
                d.qpos[:16]=ticks_to_sim(actual);d.qvel[:16]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_step(m,d)
                if i%5==0:mujoco.mj_forward(m,d);maxcp=max(maxcp,observe(d)['cube_overlap_mm'])
            mujoco.mj_forward(m,d);last=observe(d);B=d.xmat[cb].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip((-B[:,1])@up,-1,1))));shift=float(1000*np.linalg.norm(d.qpos[-7:-4]-initial[:3]));ok=bool(maxcp<=1 and tilt<=15 and shift<=12 and last['contacts'] and not last['external_cube_contact']);row.update(settled_contacts=last,max_cube_overlap_mm=maxcp,green_up_error_deg=tilt,settle_translation_mm=shift,model_feasible=ok,settled_cube_qpos=d.qpos[-7:].tolist())
            if ok:options.append((lift,d.qpos[-7:].copy(),row))
        else:row['model_feasible']=False
        rows.append(row)
    review=dict(status='legacy_P4_bounded_height_seating_diagnostic',source_photo=photo['folder']+'/'+photo['image'],source_center_palm_m=cp.tolist(),cases=rows,hand_registration_approximate=True,height_offsets_are_uncertainty_hypotheses_not_measured_corrections=True,hardware_execution_allowed=False,real_devices_opened=False);(OUT/'seed_screen.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    if options:
        lift,state,row=min(options,key=lambda v:v[0]);seed=dict(source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),initial_cube_pose_hypothesis='two-face original P4 photo plus explicit bounded height/settling hypothesis',actual_hand_ticks=actual.tolist(),commanded_hand_ticks=goal.tolist(),response_residual_ticks=(actual-goal).tolist(),settled_cube_qpos=state.tolist(),up_world=up.tolist(),up_cube_normal=[0,-1,0],up_face='green',source_height_uncertainty_offset_mm=lift,placement_and_friction_assumed=True,hardware_execution_allowed=False,real_devices_opened=False);(OUT/'seed.json').write_text(json.dumps(seed,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review,indent=2),flush=True)
if __name__=='__main__':main()
