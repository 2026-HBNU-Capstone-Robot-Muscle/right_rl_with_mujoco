"""Save the evolved nominal state of the completed new grasp; source untouched."""
from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_natural_grasp_hold_20261030.py').read_text(encoding='utf-8');anchor='                if stopped:break\n';assert s.count(anchor)==1
insert='''                if sample==forward_end and stopped is None:
                    mujoco.mj_saveModel(m,str(out/'grasp_checkpoint.mjb'))
                    np.savez_compressed(out/'grasp_checkpoint.npz',qpos=d.qpos,qvel=d.qvel,ctrl=d.ctrl,act=d.act,qacc_warmstart=d.qacc_warmstart,time=d.time,up=up,initial_cube_rotation=B,initial_cube_center=C)
                    contacts=[]
                    for ci,con in enumerate(d.contact):
                        aa,bb=int(con.geom1),int(con.geom2)
                        if cube not in (aa,bb):continue
                        other=bb if aa==cube else aa;f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f)
                        contacts.append(dict(geom=m.geom(other).name,normal_force_N=float(f[0]),depth_mm=-float(con.dist)*1000,position=con.pos.tolist(),normal=con.frame[:3].tolist(),cube_is_geom1=aa==cube))
                    report=dict(status='evolved_natural_grasp_checkpoint_only',sample=sample,grasp_yaw_from_placement_deg=angle(d.xmat[m.body('cube').id].reshape(3,3),B),cube_center=d.qpos[-7:-4].tolist(),contacts=contacts,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,initial_placement_passed=placement_ok,source_candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_model_sha256=hashlib.sha256((ROOT/'screen_natural_grasp_hold_20261030.py').read_bytes()).hexdigest(),hardware_execution_allowed=False,motor_writes=0)
                    (out/'checkpoint_review.json').write_text(json.dumps(report,indent=2)+'\\n',encoding='utf-8');print(json.dumps(report),flush=True);return
'''
(R/'capture_natural_grasp_checkpoint_20261030.py').write_text(s.replace(anchor,insert+anchor),encoding='utf-8');print('Separate grasp checkpoint probe created. Frozen source unchanged.')
