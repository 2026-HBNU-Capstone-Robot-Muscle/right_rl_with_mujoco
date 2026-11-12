from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_lower_body_free_20261030.py').read_text(encoding='utf-8');s=s.replace('yaw_goal=yaw[-1]','yaw_goal=(yaw[-1] if len(yaw) else grasp[-1])');anchor='                if stopped:break\n';assert s.count(anchor)==1
insert='''                if sample==int(data['body_hold_end_sample']) and stopped is None:
                    mujoco.mj_saveModel(m,str(out/'grasp_checkpoint.mjb'))
                    np.savez_compressed(out/'grasp_checkpoint.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=B,initial_cube_center=C)
                    contacts=[]
                    for ci,con in enumerate(d.contact):
                        aa,bb=int(con.geom1),int(con.geom2)
                        if cube not in (aa,bb):continue
                        other=bb if aa==cube else aa;f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);contacts.append(dict(geom=m.geom(other).name,normal_force_N=float(f[0]),depth_mm=-float(con.dist)*1000))
                    report=dict(status='evolved_natural_lower_body_grasp_checkpoint',sample=sample,grasp_yaw_from_placement_deg=angle(d.xmat[m.body('cube').id].reshape(3,3),B),contacts=contacts,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,body_hold_max_yaw_drift_deg=body_drift,body_hold_max_center_shift_mm=body_shift,body_contact_fraction={k:v/body_steps if body_steps else 0 for k,v in body_counts.items()},initial_placement_passed=placement_ok,hardware_execution_allowed=False,motor_writes=0)
                    (out/'checkpoint_review.json').write_text(json.dumps(report,indent=2)+'\\n',encoding='utf-8');print(json.dumps(report),flush=True);return
'''
(R/'capture_natural_lower_checkpoint_20261030.py').write_text(s.replace(anchor,insert+anchor),encoding='utf-8');print('Independent lower-contact free-body checkpoint probe created.')
