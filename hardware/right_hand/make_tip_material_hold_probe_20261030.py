from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'screen_yaw_roundtrip_model_20261030.py').read_text(encoding='utf-8')
a='args=parser.parse_args()';assert s.count(a)==1;s=s.replace(a,"parser.add_argument('--tip-friction',type=float,choices=(.3,.7,1.),required=True);args=parser.parse_args()")
a='material=apply_material(m,friction,hand,cube)';assert s.count(a)==1;s=s.replace(a,a+";m.geom_friction[[m.geom('mf_tip').id,m.geom('th_tip').id],0]=args.tip_friction")
anchor='                if stopped:break\n';assert s.count(anchor)==1
block='''                if sample==forward_end and stopped is None:
                    saved={k:getattr(d,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};saved_time=d.time;heldR=d.xmat[m.body('cube').id].reshape(3,3).copy();heldC=d.qpos[-7:-4].copy();probe_reports=[];cube_body=m.body('cube').id
                    for torque in (0.,.005,-.005,.01,-.01):
                        for k,v in saved.items():getattr(d,k)[:]=v
                        d.time=saved_time;d.xfrc_applied[:]=0;mujoco.mj_forward(m,d);drift=shift=sm=su=cp=tr=0.;tip_counts={'mf_tip':0,'th_tip':0};heights={'mf_tip':[],'th_tip':[]};stop_probe=None
                        for step in range(3000):
                            d.xfrc_applied[cube_body,3:]=up*(torque if step<1000 else 0.);mujoco.mj_step(m,d);sup,selfc=inspect(d);sm=max(sm,max(selfc.values(),default=0.));su=max(su,max(sup.values(),default=0.));tr=max(tr,float(max(abs(d.qpos[:16]-d.ctrl))));Rnow=d.xmat[cube_body].reshape(3,3);drift=max(drift,abs(angle(Rnow,heldR)));shift=max(shift,float(np.linalg.norm(d.qpos[-7:-4]-heldC)*1000));normal={}
                            for ci,con in enumerate(d.contact):
                                aa,bb=int(con.geom1),int(con.geom2)
                                if cube not in (aa,bb):continue
                                other=bb if aa==cube else aa;name=m.geom(other).name
                                if int(m.geom_bodyid[other]) in hand:cp=max(cp,max(0.,-float(con.dist)*1000))
                                if name in tip_counts:
                                    force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);normal[name]=normal.get(name,0.)+float(force[0]);heights[name].append(float((con.pos-d.qpos[-7:-4])@(Rnow@np.array([0.,-1.,0]))*1000))
                            for name in tip_counts:tip_counts[name]+=normal.get(name,0.)>.2
                            if sm>1e-6 or su>.15 or cp>1 or tr>.18:stop_probe='unchanged geometry/tracking bound';break
                        row=dict(torque_Nm=torque,torque_is_design_probe_not_measured_layer_load=True,max_yaw_drift_deg=drift,max_center_shift_mm=shift,max_self_mm=sm,max_support_mm=su,max_cube_mm=cp,max_tracking_rad=tr,stop=stop_probe,steps=step+1,
                            opposing_tip_contact_fraction={k:v/(step+1) for k,v in tip_counts.items()},tip_contact_height_range_mm={k:[min(v),max(v)] if v else None for k,v in heights.items()},
                            stationary_grasp_probe_passed=bool(stop_probe is None and drift<5 and shift<3 and all(v/(step+1)>.8 for v in tip_counts.values())))
                        probe_reports.append(row);print(json.dumps(row),flush=True)
                    result=dict(status='tip_material_stationary_grasp_sensitivity_only',tip_sliding_coefficient=args.tip_friction,other_hand_and_cube_sliding_coefficient=friction,material_coefficients_not_physically_identified=True,
                        modeled_forward_from_placement_deg=forward_whole_turn,cube_has_remained_free_during_each_probe=True,separate_probes_restore_same_evolved_forward_checkpoint=True,
                        probes=probe_reports,all_hold_probes_passed=all(v['stationary_grasp_probe_passed'] for v in probe_reports),shape_gains_and_force_limits_unchanged=True,
                        source_candidate_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),actual_upper_layer_turn_not_verified=True,hardware_execution_allowed=False,devices_opened=False)
                    (out/'hold_sensitivity_review.json').write_text(json.dumps(result,indent=2)+'\\n',encoding='utf-8');print('HOLD PROBES',args.tip_friction,result['all_hold_probes_passed'],flush=True);return
'''
(R/'probe_tip_material_and_hold_20261030.py').write_text(s.replace(anchor,block+anchor),encoding='utf-8');print('Created material-only holding sensitivity probe; frozen source unchanged.')
