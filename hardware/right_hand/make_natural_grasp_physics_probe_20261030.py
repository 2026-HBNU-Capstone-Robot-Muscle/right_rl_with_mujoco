"""Prepare a free-body grasp/hold/release probe; no device or hardware admission."""
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/stable_support_research/retargeted_grasp_native';s=np.load(O/'candidate.npz');close=s['goals'][s['phases']=='stage_5_loaded_grasp'];hold=np.tile(close[-1],(67,1));release=np.vstack([close[::-1],s['start']]);q=np.vstack([close,hold,release,np.tile(s['start'],(20,1))]);p=np.array(['stage_5_loaded_grasp']*len(close)+['stage_6_grasp_hold']*len(hold)+['stage_9_loaded_open']*len(release)+['stage_10_remove_hold']*20);assert np.max(abs(np.diff(np.vstack([s['start'],q]),axis=0)))<=2;np.savez_compressed(O/'grasp_hold_release.npz',start=s['start'],goals=q,phases=p,forward_end_sample=np.array(len(close)+len(hold)))
text=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8');text=text.replace('yaw_goal=yaw[-1]','yaw_goal=(yaw[-1] if len(yaw) else grasp[-1])')
anchor="forward_end=int(data['forward_end_sample'])";assert text.count(anchor)==1;text=text.replace(anchor,anchor+";grasp_R=None;grasp_C=None;grasp_steps=0;grasp_eval_steps=0;grasp_drift=grasp_shift=0.;grasp_counts={'mf_tip':0,'th_tip':0}")
anchor="                    if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25:stopped='contact/range/tracking/retention violation';break"
assert text.count(anchor)==1
insert='''                    if phase=='grasp_hold':
                        grasp_steps+=1
                        if grasp_steps>=round(1./args.dt):
                            if grasp_R is None:grasp_R=d.xmat[m.body('cube').id].reshape(3,3).copy();grasp_C=d.qpos[-7:-4].copy()
                            grasp_eval_steps+=1;grasp_drift=max(grasp_drift,abs(angle(d.xmat[m.body('cube').id].reshape(3,3),grasp_R)));grasp_shift=max(grasp_shift,float(np.linalg.norm(d.qpos[-7:-4]-grasp_C)*1000));normal={}
                            for ci,con in enumerate(d.contact):
                                aa,bb=int(con.geom1),int(con.geom2)
                                if cube not in (aa,bb):continue
                                other=bb if aa==cube else aa;name=m.geom(other).name
                                if name in grasp_counts:
                                    force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0])
                            for name in grasp_counts:grasp_counts[name]+=normal.get(name,0.)>.2
'''
text=text.replace(anchor,insert+anchor)
anchor="            row=dict(vertical_seating_line_upper_verified_free=";assert text.count(anchor)==1
text=text.replace(anchor,"            grasp_task=dict(evaluated_steps=grasp_eval_steps,max_yaw_drift_deg=grasp_drift,max_center_shift_mm=grasp_shift,opposing_contact_fraction={k:v/grasp_eval_steps if grasp_eval_steps else 0 for k,v in grasp_counts.items()},passed=bool(grasp_eval_steps and grasp_drift<3 and grasp_shift<2 and all(v/grasp_eval_steps>.8 for v in grasp_counts.values())))\n            row=dict(grasp_hold_observation=grasp_task,vertical_seating_line_upper_verified_free=")
text=text.replace("status='continuous_forward_inverse_full_loaded_model_diagnostic'","status='natural_cradle_free_grasp_hold_release_diagnostic'")
(R/'screen_natural_grasp_hold_20261030.py').write_text(text,encoding='utf-8');print('Prepared',len(q),'goals; zero yaw commands; original physics/safety bounds preserved.')
