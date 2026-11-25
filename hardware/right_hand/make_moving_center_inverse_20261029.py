"""Create a distinct moving-center contact hypothesis; preserve failed study."""
from pathlib import Path
R=Path(__file__).resolve().parent;s=(R/'propose_observed_inverse_tail_20261029.py').read_text(encoding='utf-8')
changes=[
("OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_tail'","OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_moving_center'"),
("for nw in (0.,2.,5.):","for nw in (2.,):"),
("previous=actual.copy();previous[13]-=128;previous_command=goal.copy();previous_command[13]-=128","previous=actual.copy();previous[13]-=128;previous_command=goal.copy();previous_command[13]-=128;center_shift=np.zeros(2)"),
("center=C+up*(height_mm/1000)*min(1.,abs(angle)/15)","center0=C+up*(height_mm/1000)*min(1.,abs(angle)/15)"),
("raw=previous.copy();raw[ACTIVE]=x/TICK+2048;d.qpos[:16]=ticks_to_sim(raw);mujoco.mj_forward(m,d)","raw=previous.copy();raw[ACTIVE]=x[:8]/TICK+2048;center=center0+R0[:,[0,2]]@x[8:]/1000;d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=center;qrot=Rotation.from_matrix(R).as_quat();d.qpos[-4:]=qrot[[3,0,1,2]];mujoco.mj_forward(m,d)"),
("res.extend((x-(previous[ACTIVE]-2048)*TICK)*.3);res.extend([selfmm*100,supmm*100])","res.extend((x[:8]-(previous[ACTIVE]-2048)*TICK)*.3);res.extend([selfmm*100,supmm*100]);res.extend(x[8:]*.03);cp=0.\n                for contact in d.contact:\n                    a,b=int(contact.geom1),int(contact.geom2)\n                    if cube in (a,b):\n                        other=b if a==cube else a\n                        if (m.geom(other).name or '') not in ('mf_tip','th_tip'):cp=max(cp,max(0.,-float(contact.dist)*1000))\n                res.append(cp*20)"),
("actual=raw.tolist()) if detail", "actual=raw.tolist(),center_shift_mm=x[8:].tolist(),other_cube_depth_mm=cp) if detail"),
("x=np.clip((seed[ACTIVE]-2048)*TICK,lower+1e-8,upper-1e-8)","x=np.r_[np.clip((seed[ACTIVE]-2048)*TICK,lower+1e-8,upper-1e-8),center_shift]"),
("bounds=(lower,upper),max_nfev=130", "bounds=(np.r_[lower,-12,-12],np.r_[upper,12,12]),max_nfev=180,x_scale='jac'"),
("command=np.rint(raw+offset).astype(int)", "command=np.rint(raw+offset).astype(int);center_shift=sol.x[8:].copy()"),
("info['support_mm']>.15 or", "info['support_mm']>.15 or info['other_cube_depth_mm']>1. or"),
]
for a,b in changes:
 if s.count(a)!=1:raise RuntimeError((a,s.count(a)))
 s=s.replace(a,b)
(R/'propose_moving_center_inverse_20261029.py').write_text(s,encoding='utf-8')
print('Created separate moving-center research proposer; no robot access.')
