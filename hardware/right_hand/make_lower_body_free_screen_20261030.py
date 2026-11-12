"""Instrument frozen source for the additional release/reseat/close/hold stages."""
from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8')
a="'stage_10_remove_hold':'remove_hold'";assert s.count(a)==1;s=s.replace(a,a+",'stage_11_body_release':'body_release','stage_12_body_reseat':'body_reseat','stage_13_body_close':'body_close','stage_14_body_hold':'body_hold'")
s=s.replace('forward_end=int(data[\'forward_end_sample\'])',"forward_end=int(data['forward_end_sample']);body_hold_start=None;body_hold_R=None;body_hold_C=None;body_drift=body_shift=0.;body_counts={'mf_tip':0,'th_tip':0};body_heights={'mf_tip':[],'th_tip':[]};body_steps=0")
a='                    if maxself>1e-6 or maxsup>.15';assert s.count(a)==1
insert='''                    if phase=='body_hold':
                        if body_hold_R is None:body_hold_R=d.xmat[m.body('cube').id].reshape(3,3).copy();body_hold_C=d.qpos[-7:-4].copy()
                        Rbody=d.xmat[m.body('cube').id].reshape(3,3);body_drift=max(body_drift,abs(angle(Rbody,body_hold_R)));body_shift=max(body_shift,float(np.linalg.norm(d.qpos[-7:-4]-body_hold_C)*1000));body_steps+=1;normal={}
                        for ci,con in enumerate(d.contact):
                            aa,bb=int(con.geom1),int(con.geom2)
                            if cube not in (aa,bb):continue
                            other=bb if aa==cube else aa;name=m.geom(other).name
                            if name in body_counts:
                                force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0]);body_heights[name].append(float((con.pos-d.qpos[-7:-4])@(Rbody@np.array([0.,-1.,0]))*1000))
                        for name in body_counts:body_counts[name]+=normal.get(name,0.)>.2
'''
s=s.replace(a,insert+a)
a=next(line for line in s.splitlines() if 'row=dict(vertical_seating_line_upper_verified_free=' in line);indent=a[:len(a)-len(a.lstrip())];assert s.count(a)==1
s=s.replace(a,indent+"body_hold=dict(max_yaw_drift_deg=body_drift,max_center_shift_mm=body_shift,samples=body_steps,opposing_contact_fraction={k:v/body_steps if body_steps else 0 for k,v in body_counts.items()},tip_height_range_mm={k:[min(v),max(v)] if v else None for k,v in body_heights.items()},hold_task_passed=bool(body_steps and body_drift<5 and body_shift<3 and all(v/body_steps>.8 for v in body_counts.values()) and all(v and max(v)<.055/6*1000 for v in body_heights.values())))\n"+a.replace('row=dict(','row=dict(body_hold_observation=body_hold,'))
s=s.replace("status='continuous_forward_inverse_full_loaded_model_diagnostic'","status='continuous_forward_then_lower_body_regrasp_model_diagnostic'")
(R/'screen_lower_body_free_20261030.py').write_text(s,encoding='utf-8');print('Created separate full free-cube screen; frozen source unchanged.')
