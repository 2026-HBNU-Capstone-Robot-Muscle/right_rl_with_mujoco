"""New loaded joint-route dynamics; the cube is disabled for hand clearance.

Historical pitch success and new free-cube transfer are separate evidence.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,time
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,ticks_to_sim
from book_empty_preparation_plan import FRONTS
from screen_pitch_yaw_join_20261028 import contact_reader
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify as classify_seam,metadata as seam_metadata,PAIR as SEAM_PAIR
from pitch_handoff_plan_20261028 import OUT,LOADED_PATH,TRANSFER_SEED

def main():
    folder=OUT/'loaded_hand_dynamics';folder.mkdir(parents=True,exist_ok=True)
    z=np.load(LOADED_PATH);start=z['start'];goals=z['goals'];before=time.perf_counter()
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg);lo,hi=limits();ids=np.asarray(SIM_TO_MOTOR_IDS)-1
    up=np.asarray(json.loads(TRANSFER_SEED.read_bytes())['up_world'])
    low,high=(lo[ids]-2048)*TICK,(hi[ids]-2048)*TICK;results=[]
    for front in FRONTS:
        m,manifest=build(env,folder/f'scene_{front:g}.xml',book_projection_mm=front)
        correct_model(m,start);m.opt.timestep=.005;m.opt.gravity[:]=-9.81*up
        mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
        inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias
            m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16]
            d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
            self_mm=support_mm=tracking=excess=raw_self=discarded=0.;steps=0;stopped=None;worst_pair=None
            rows=np.vstack([np.repeat(start[None,:],10,axis=0),goals,np.repeat(goals[-1][None,:],20,axis=0)])
            for i,q in enumerate(rows,1):
                d.ctrl[:]=ticks_to_sim(q);target=round(i*.075/.005)
                for _ in range(target-steps):
                    mujoco.mj_step(m,d);su,sp=inspect(d)
                    raw_self=max(raw_self,max(sp.values(),default=0.))
                    filtered=classify_seam(sp)
                    if SEAM_PAIR in sp and SEAM_PAIR not in filtered:discarded=max(discarded,sp[SEAM_PAIR])
                    if filtered and max(filtered.values())>self_mm:worst_pair=max(filtered,key=filtered.get)
                    self_mm=max(self_mm,max(filtered.values(),default=0.));support_mm=max(support_mm,max(su.values(),default=0.))
                    tracking=max(tracking,float(max(abs(d.qpos[:16]-d.ctrl))))
                    excess=max(excess,float(max(0.,max(low-d.qpos[:16]),max(d.qpos[:16]-high))))
                    if self_mm>1e-6 or support_mm>.15 or tracking>.18 or excess>1e-6:
                        stopped='hand_contact_range_or_tracking';break
                steps=target
                if stopped:break
            row=dict(book_front_mm=front,gain_scale=scale,passed=stopped is None,stop=stopped,
                     max_self_mm=self_mm,max_support_mm=support_mm,max_tracking_rad=tracking,
                     unapproved_range_excess_rad=excess,samples_completed=i,
                     raw_max_self_mm=raw_self,existing_palm7_seam_discarded_max_mm=discarded,worst_unclassified_pair=worst_pair)
            results.append(row);print(json.dumps(row),flush=True)
    report=dict(status='pitch_and_transfer_finite_hand_dynamics',source_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),
                cases=results,all_passed=all(r['passed'] for r in results),cube_disabled=True,
                existing_palm7_seam_policy=seam_metadata(),gravity_up_world=up.tolist(),
                finite_actuators=True,hand_qpos_prescribed=False,hardware_execution_allowed=False,devices_opened=False,
                elapsed_seconds=time.perf_counter()-before)
    (folder/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
