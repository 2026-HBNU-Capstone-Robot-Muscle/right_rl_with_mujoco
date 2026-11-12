from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8');anchor='            for sample,(phase,q) in enumerate(zip(phases,goals),1):';assert s.count(anchor)==1;s=s.replace(anchor,"            task_trace=[];best_score=float('inf');best_sample=None;best_state=None;best_angle=None;best_tilt=None\n"+anchor)
anchor='                if stopped:break';assert s.count(anchor)==1
insert='''                current_R=d.xmat[m.body('cube').id].reshape(3,3);current_angle=angle(current_R,B);current_tilt=float(np.degrees(np.arccos(np.clip(current_R[:,1]@B[:,1],-1,1))));score=abs(current_angle-90)+10*max(0.,current_tilt-15)
                if score<best_score:
                    best_score=score;best_sample=sample;best_angle=current_angle;best_tilt=current_tilt;best_state={key:getattr(d,key).copy() for key in ('qpos','qvel','ctrl','act','qacc_warmstart')};best_state.update(time=d.time,up=up,initial_cube_rotation=B.copy(),initial_cube_center=C.copy())
                if sample%16==0 or sample==len(goals) or stopped:task_trace.append(dict(sample=sample,phase=str(phase),whole_yaw_deg=current_angle,tilt_from_placement_deg=current_tilt,goal=q.tolist()))
'''
s=s.replace(anchor,insert+anchor);anchor='            row=dict(';assert s.count(anchor)==1;s=s.replace(anchor,"            if best_state is not None:np.savez_compressed(out/'closest_state.npz',**best_state);mujoco.mj_saveModel(m,str(out/'closest_model.mjb'))\n"+"            row=dict(best_sample=best_sample,best_whole_yaw_deg=best_angle,best_tilt_from_placement_deg=best_tilt,task_trace=task_trace,")
(R/'trace_raised_cradle_free_20261104.py').write_text(s,encoding='utf-8');print('Unchanged physics derivative now records whole-cube trajectory and closest90state.')
