"""Look for a thumb fingertip opposing an index/middle fingertip, offline."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,itertools,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--seed-folder');args=parser.parse_args()
    if args.seed_folder:OUT=(ROOT/args.seed_folder).resolve();OUT.relative_to(ROOT)
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);state=np.asarray(seed['settled_cube_qpos']);C=state[:3];B=Rotation.from_quat(state[-4:][[1,2,3,0]]).as_matrix();lo,hi=limits();ids=np.array([3,2,1,0])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'thumb_opposed_pinch_scene.xml');set_cube_physics(m,.055,.1);d=mujoco.MjData(m);cube=m.geom('cube').id;tip=m.geom('th_tip').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def put(goal):d.qpos[:16]=ticks_to_sim(goal+residual);d.qpos[-7:]=state;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
    def defects():
        sp=sup=wrong=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b) and tip not in (a,b):wrong=max(wrong,dep)
        return sp,sup,wrong
    results=[]
    up_local=np.asarray(seed.get('up_cube_normal',[1,0,0]));side_axes=[a for a in range(3) if abs(up_local[a])<.5];general=seed.get('up_face')=='green'
    entries=itertools.product(side_axes,(-1,1) if general else (-1,),(0.,.01) if general else (.005,.015),(-.01,.01))
    for axis,sign,height,offset in entries:
        normal=sign*B[:,axis];up=B@up_local;tangent=np.cross(normal,up);tangent/=np.linalg.norm(tangent);target=C+normal*.0365+up*height+tangent*offset;fits=[]
        for degrees in ((30,95,80,50),(60,40,45,30),(15,120,10,0),(90,90,0,0)):
            initial=np.clip(2048+np.asarray(degrees)*4096/360,lo[ids]+.01,hi[ids]-.01)
            def objective(value):
                q=base.astype(float).copy();q[ids]=value;put(q);p=d.geom_xpos[tip];gap=mujoco.mj_geomDistance(m,d,cube,tip,1.,None)*1000;sp,sup,wrong=defects()
                return np.r_[1000*np.array([(p-target)@tangent,(p-target)@up]),10*(gap-.1),15*sp,15*sup,15*max(0,wrong-.8),5*max(0,.0275-(p-C)@normal)*1000,.001*(value-base[ids])]
            fit=least_squares(objective,initial,bounds=(lo[ids]+.001,hi[ids]-.001),diff_step=.00004,max_nfev=180);q=base.copy();q[ids]=np.rint(fit.x).astype(int);put(q);p=d.geom_xpos[tip].copy();gap=float(mujoco.mj_geomDistance(m,d,cube,tip,1.,None)*1000);sp,sup,wrong=defects();error=float(1000*np.linalg.norm([(p-target)@tangent,(p-target)@up]))
            fits.append(dict(axis=axis,sign=sign,height_mm=height*1000,tangent_offset_mm=offset*1000,goal_ticks=q.tolist(),gap_mm=gap,position_error_mm=error,self_overlap_mm=sp,support_overlap_mm=sup,other_cube_overlap_mm=wrong,
                endpoint_promising=bool(-.3<=gap<=2 and error<=6 and sp<=.01 and sup<=.15 and wrong<=1)))
        row=min(fits,key=lambda v:abs(v['gap_mm'])+v['position_error_mm']+20*v['self_overlap_mm']+20*v['support_overlap_mm']+10*v['other_cube_overlap_mm']);results.append(row);print('THUMB',json.dumps(row),flush=True)
    fingers=json.loads((OUT/'pinch_upper_corner_endpoints.json').read_bytes())['candidates'];pairs=[]
    for th,f in itertools.product([v for v in results if v['endpoint_promising']],[v for v in fingers if v['endpoint_promising']]):
        if th['axis']!=f['side_axis'] or f['side_sign']!=-th['sign']:continue
        fid=[7,5,6,4] if f['finger']=='if' else [11,9,10,8];q=np.asarray(th['goal_ticks']);q[fid]=np.asarray(f['goal_ticks'])[fid];put(q);sp,sup,wrong=defects();gaps={n:float(mujoco.mj_geomDistance(m,d,cube,m.geom(n+'_tip').id,1.,None)*1000) for n in ('th',f['finger'])};pairs.append(dict(thumb=th,finger=f['finger'],finger_candidate=f,goal_ticks=q.tolist(),gaps_mm=gaps,self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=wrong,endpoint_promising=bool(sp<=.01 and sup<=.15 and wrong<=1 and all(-.3<=v<=2 for v in gaps.values()))))
    report=dict(status='thumb_opposed_fingertip_endpoints_only',thumb_candidates=results,pairs=pairs,model=manifest,transition_not_screened=True,free_cube_grasp_not_verified=True,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'thumb_opposed_pinch_endpoints.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('SUMMARY',dict(thumb_promising=sum(v['endpoint_promising'] for v in results),paired_promising=sum(v['endpoint_promising'] for v in pairs)),flush=True)
if __name__=='__main__':main()
