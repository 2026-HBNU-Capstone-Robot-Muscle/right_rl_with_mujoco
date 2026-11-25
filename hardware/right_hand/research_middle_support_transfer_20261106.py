"""Propose a middle proximal support before repositioning the index finger.

Cube pinning is confined to static proposals. Actual support acquisition is
tested by position-goal replay with the cube free and all dynamic guards active.
"""
import os
os.environ['JAX_PLATFORMS']='cpu'
import argparse,hashlib,json,time
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.stats import qmc
from dynamic_common_grip_search_20261105 import ROOT,R,ramp
from research_rotation_body_hold_20261104 import restore,inspect_for,native,motor
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK
from upper_layer_cylinder_clearance_20261105 import precompute,gaps
from canonical_grip_dynamics_20261105 import run

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--nearest-feature',action='store_true');ap.add_argument('--contact-geom',choices=('mf_px_collision','mf_tip'),default='mf_px_collision');ap.add_argument('--future-side-mm',type=float,choices=(0.,8.),default=0.);args=ap.parse_args()
    parent=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());model=R/parent['source_model'];assert sha(model)==parent['source_model_sha256'];m=mujoco.MjModel.from_binary_path(str(model));source=ROOT/'countergrip_level/seed0_source1_lift2_normal1.5_n850/verified_end_state.npz';z=np.load(source);d=restore(m,z);base=d.qpos.copy();pd=d.ctrl-base[:16];cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy();hand,_,check=inspect_for(m);geometry=precompute(m,hand);mf_geometry=[v for v in geometry if m.body(int(m.geom_bodyid[v[0]])).name.startswith('mf_')];vertices={g:v for g,v,e in geometry};gid=m.geom(args.contact_geom).id;normal=B[:,1];active=np.arange(4,8);lo,hi,nlo,nhi=native(m);ids=np.asarray(SIM_TO_MOTOR_IDS)-1;low=np.maximum((np.ceil(lo[ids])+8-2048)*TICK-pd,nlo)[active];high=np.minimum((np.floor(hi[ids])-8-2048)*TICK-pd,nhi)[active];tag=('middle_tip_support_transfer' if args.contact_geom=='mf_tip' else 'middle_support_transfer')+('_nearest' if args.nearest_feature else '');out=ROOT/tag;out.mkdir(exist_ok=True);start=motor(z['ctrl']);began=time.perf_counter()
    if args.future_side_mm:
        C=C+np.array([0.,args.future_side_mm/1000,0.]);out=ROOT/(tag+f'_futureY{args.future_side_mm:g}');out.mkdir(exist_ok=True)
    def measure(x,full=False):
        d.qpos[:]=base;d.qpos[-7:-4]=C;d.qpos[active]=x;d.ctrl[:]=d.qpos[:16]+pd;d.qvel[:]=0;mujoco.mj_forward(m,d);world=vertices[gid]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid];proj=(world-C)@normal;weights=np.exp(-(proj-min(proj))/.0002);point=np.average(world,axis=0,weights=weights);local=(point-C)@B*1000;gap=float(min(proj)-.0275)*1000
        if args.nearest_feature:
            line=np.zeros(6);gap=float(mujoco.mj_geomDistance(m,d,gid,cube,.15,line))*1000;point=line[:3];local=(line[3:]-C)@B*1000
        if not full:return gap,local,point
        su,sp=check(d);cl=gaps(m,d,C,B,mf_geometry,True);mf_ids={v[0] for v in mf_geometry};pen=max((max(0.,-float(c.dist)*1000) for c in d.contact if cube in (int(c.geom1),int(c.geom2)) and (not args.future_side_mm or (int(c.geom2) if int(c.geom1)==cube else int(c.geom1)) in mf_ids)),default=0.)
        return gap,local,point,min(v for name,v in cl),max(sp.values(),default=0.),max(su.values(),default=0.),pen
    samples=qmc.scale(qmc.Sobol(4,scramble=True,seed=261106).random_base2(11),low,high);samples=np.vstack([np.clip(base[active],low+1e-9,high-1e-9),samples]);scores=[]
    for x in samples:
        gap,p,point=measure(x);scores.append((gap+.03)**2*10+((p[1]-27.5)**2*10 if args.nearest_feature else 0)+max(0.,abs(p[0])-23)**2*2+max(0.,abs(p[2])-23)**2*2+((p[0]-20)**2+(p[2]+20)**2)*.03)
    np.savez_compressed(out/'sampled_workspace.npz',joint_proposals=samples,scores=scores);selected=[]
    for index in np.argsort(scores):
        if all(np.linalg.norm(samples[index]-p)>.3 for p in selected):selected.append(samples[index])
        if len(selected)>=6:break
    results=[]
    for seed_index,seed in enumerate(selected):
        folder=out/f'seed{seed_index}';folder.mkdir(exist_ok=True)
        def residual(x):
            gap,p,point,clear,selfmm,supportmm,pen=measure(x,True)
            return np.r_[(gap+.03)*10,(p[1]-27.5)*10 if args.nearest_feature else 0,max(0.,abs(p[0])-23)*10,max(0.,abs(p[2])-23)*10,(p[0]-20)*.1,(p[2]+20)*.1,(x-seed)*.1,max(0.,.8-clear)*20,selfmm*500000,supportmm*1000,max(0.,pen-.8)*1000]
        fit=least_squares(residual,np.clip(seed,low+1e-9,high-1e-9),bounds=(low,high),max_nfev=150,ftol=1e-9,xtol=1e-9,gtol=1e-9);gap,p,point,clear,selfmm,supportmm,pen=measure(fit.x,True);footprint=abs(p[1]-27.5)<.3 and max(abs(p[0]),abs(p[2]))<=27.50001 if args.nearest_feature else abs(p[0])<26 and abs(p[2])<26;accepted=bool(abs(gap)<.2 and footprint and clear>.3 and selfmm<=1e-6 and supportmm<=.15 and pen<=1);np.savez_compressed(folder/'static_proposal.npz',actual_joint_proposal=d.qpos[:16],cube_center=C,cube_rotation=B);row=dict(seed=seed_index,static_accepted=accepted,bottom_gap_mm=gap,point_cube_local_mm=p.tolist(),middle_only_upper_gap_mm=clear,self_mm=selfmm,support_mm=supportmm,cube_mm=pen,dynamic=[])
        if accepted:
            actual=d.qpos[:16].copy();jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,point,int(m.geom_bodyid[gid]));bias=d.qfrc_bias[active]-d.qfrc_passive[active]
            for force in ((0.,.3) if args.future_side_mm else (.3,.7,1.)):
                control=actual+pd;control[active]=actual[active]+(bias-(normal@jp[:,active])*force)/m.actuator_gainprm[active,0];goal=motor(control)
                if np.any(goal<lo) or np.any(goal>hi):row['dynamic'].append(dict(force_N=force,native_goal_rejected=True));continue
                rows=ramp(start,goal,134);np.savez_compressed(folder/f'force{force:g}.npz',start=start,goals=rows);v=run(m,z,rows,1,folder/f'force{force:g}_end.npz',None,True,5,('if_tip','th_tip'));row['dynamic'].append(dict(force_N=force,result=v))
        results.append(row);report=dict(status='middle_support_proposals_and_free_replays',contact_geom=args.contact_geom,future_side_shift_for_static_proposal_mm=args.future_side_mm,nonmiddle_future_cube_contacts_excluded_only_from_static_proposal=True if args.future_side_mm else False,source_state=source.relative_to(R).as_posix(),source_state_sha256=sha(source),source_model_sha256=sha(model),cases=results,static_middle_clearance_not_whole_hand_clearance=True,cube_pinned_only_for_proposals=True,free_cube_never_reset_during_replay=True,native_geometry_force_caps_and_dynamic_guards_unchanged=True,devices_opened=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began);(out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(seed=seed_index,static=accepted,gap=gap,point=p.tolist(),middle_clearance=clear,dynamic=[dict(force=t['force_N'],safety=t.get('result',{}).get('safety_passed'),yaw=t.get('result',{}).get('final_yaw_from_placement_deg'),tilt=t.get('result',{}).get('final_up_tilt_deg'),middle_N=t.get('result',{}).get('final_contact_N',{}).get(args.contact_geom,0)) for t in row['dynamic']])),flush=True)
if __name__=='__main__':main()
