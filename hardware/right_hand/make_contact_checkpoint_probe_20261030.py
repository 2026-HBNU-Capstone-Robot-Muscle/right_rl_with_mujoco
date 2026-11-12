"""Instrument the frozen physics source at a single previously safe forward endpoint."""
from pathlib import Path
R=Path(__file__).resolve().parent
s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8')
anchor='                if stopped:break\n'
assert s.count(anchor)==1
insert='''                if sample==forward_end and stopped is None:
                    mujoco.mj_saveModel(m,str(out/'forward_checkpoint.mjb'))
                    np.savez_compressed(out/'forward_checkpoint.npz',qpos=d.qpos,qvel=d.qvel,act=d.act,ctrl=d.ctrl,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time),up=up)
                    contacts=[]
                    for ci,c in enumerate(d.contact):
                        aa,bb=int(c.geom1),int(c.geom2)
                        if cube not in (aa,bb):continue
                        force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force)
                        other=bb if aa==cube else aa
                        contacts.append(dict(geom=m.geom(other).name,body=m.body(int(m.geom_bodyid[other])).name,depth_mm=-float(c.dist)*1000,normal_force_N=float(force[0]),position=c.pos.tolist(),frame=c.frame.tolist()))
                    result=dict(status='forward_contact_checkpoint_captured_only',sample=sample,forward_from_placement_deg=forward_whole_turn,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,
                        cube_contacts=contacts,initial_placement_passed=placement_ok,candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),base_source_sha256=hashlib.sha256((ROOT/'screen_yaw_roundtrip_model_20261030.py').read_bytes()).hexdigest(),
                        snapshot_files=['forward_checkpoint.mjb','forward_checkpoint.npz'],simulation_timestep_s=args.dt,friction=friction,gain_scale=scale,book_front_mm=front,source_proof_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
                    (out/'checkpoint_review.json').write_text(json.dumps(result,indent=2)+'\\n',encoding='utf-8');print(json.dumps(result),flush=True);return
'''
(R/'screen_capture_yaw_contact_checkpoints_20261030.py').write_text(s.replace(anchor,insert+anchor),encoding='utf-8')
print('Created diagnostic checkpoint probe; original physics source unchanged.')
