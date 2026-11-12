"""Prescribe recorded real joint readings with a camera-derived cube hypothesis."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/saved_marker_registration';RUN=ROOT/'loaded_repeat_20261026_170932_163649'
def main():
    r=json.loads((RUN/'result.json').read_bytes());registration=json.loads((OUT/'conditional_raw_frame_start_pose.json').read_bytes());p=next(v for v in registration['candidates'] if v['consistent_with_known_start']);samples=[s for s in r['samples'] if s['phase'].startswith(('stage_6_loaded','stage_7_loaded','stage_8a_loaded','stage_9_loaded','stage_10_loaded','stage_11_loaded'))]
    actual=np.asarray([s['actual'] for s in samples]);times=np.asarray([s['t'] for s in samples]);times-=times[0];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'raw_registered_replay_scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};q=Rotation.from_matrix(p['R_cube_world']).as_quat();initial=np.r_[p['cube_center_world_m'],q[[3,0,1,2]]];results=[]
    for gravity in ('palm_normal','current_table_reference'):
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual[0]);d.qpos[-7:]=initial;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);P=d.xmat[palm].reshape(3,3);up=-P[:,2] if gravity=='palm_normal' else np.asarray(registration['conditional_table_up_world']);up/=np.linalg.norm(up);m.opt.gravity[:]=-9.81*up;m.geom_friction[cube,0]=.3;start=time.perf_counter();trace=[];maxcp=maxself=maxsupport=0.;external=False;stop=None;last=0
        def observe():
            nonlocal maxcp,maxself,maxsupport,external
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
                if ha and hb:maxself=max(maxself,depth)
                if (a in support and hb) or (b in support and ha):maxsupport=max(maxsupport,depth)
                if cube in (a,b):
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:maxcp=max(maxcp,depth)
                    elif c.dist<.0004 and (other in support or m.geom(other).name=='floor'):external=True
        for step in range(int(np.ceil(times[-1]/dt))+1):
            t=step*dt
            while last+1<len(times) and times[last+1]<t:last+=1
            following=min(last+1,len(times)-1);fraction=np.clip((t-times[last])/max(1e-12,times[following]-times[last]),0,1);a=ticks_to_sim(actual[last]);b=ticks_to_sim(actual[following]);d.qpos[:16]=a+(b-a)*fraction;d.qvel[:16]=(b-a)/max(dt,times[following]-times[last]);d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            if step%5==0:
                observe()
                if maxcp>1 or external:stop='cube_contact_or_retention_model_rejected';break
            if step%50==0:trace.append(d.qpos.copy())
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);B=d.xmat[cb].reshape(3,3);red_error=float(np.degrees(np.arccos(np.clip(B[:,0]@up,-1,1))));green_front=float(np.degrees(np.arccos(np.clip((-B[:,1])@(-P[:,0]),-1,1))));name='raw_registered_replay_'+gravity+'.npz';np.savez_compressed(OUT/name,qpos=np.asarray(trace),end_qpos=d.qpos.copy(),dt_trace=.1)
        row=dict(gravity_reference=gravity,stopped=stop,simulated_seconds=step*dt,source_seconds=float(times[-1]),source_samples=len(samples),red_up_error_deg=red_error,green_wrist_error_deg=green_front,max_cube_overlap_mm=maxcp,max_self_overlap_mm=maxself,max_support_overlap_mm=maxsupport,external_cube_contact=external,end_cube_qpos=d.qpos[-7:].tolist(),conditional_forward_flip=bool(stop is None and red_error<15 and green_front<15),trace=name,elapsed_wall_seconds=time.perf_counter()-start);results.append(row);print(json.dumps(row),flush=True)
    report=dict(status='actual_joint_reading_replay_with_raw_image_cube_pose_hypothesis',source_result_sha256=hashlib.sha256((RUN/'result.json').read_bytes()).hexdigest(),source_placement_frame=registration['source'],actual_motor_readings_prescribed=True,source_time_parameter='recorded sample t',cube_initial_pose_single_face_and_approximate_hand_registration=True,friction_assumed=.3,mass_kg_assumed=.1,cases=results,model=manifest,physical_vertical_successes_remain_two=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'raw_registered_physical_flip_replay.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
