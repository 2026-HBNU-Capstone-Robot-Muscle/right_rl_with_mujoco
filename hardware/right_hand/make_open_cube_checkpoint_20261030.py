from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8');anchor="            material['effective_contacts_after_settle']=";assert s.count(anchor)==1
insert='''            mujoco.mj_saveModel(m,str(out/'open_cube_checkpoint.mjb'))
            np.savez_compressed(out/'open_cube_checkpoint.npz',qpos=d.qpos,qvel=d.qvel,act=d.act,ctrl=d.ctrl,qacc_warmstart=d.qacc_warmstart,time=np.array(d.time),up=up)
            report=dict(status='free_settled_open_hand_cube_checkpoint',initial_placement_passed=placement_ok,initial_probe=placement_probe,cube_center=d.qpos[-7:-4].tolist(),hardware_execution_allowed=False,devices_opened=False)
            (out/'checkpoint_review.json').write_text(json.dumps(report,indent=2)+'\\n',encoding='utf-8');print(json.dumps(report),flush=True);return
'''
(R/'capture_open_cube_checkpoint_offline_20261030.py').write_text(s.replace(anchor,insert+anchor),encoding='utf-8');print('Created separate initial-state diagnostic; frozen model unchanged.')
