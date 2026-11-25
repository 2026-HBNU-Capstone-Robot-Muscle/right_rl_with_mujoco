"""Fit hypothetical Cube centers to observed angles and actual encoders.

No geometry, joint range or contact threshold is changed. Fits are conditional
contact hypotheses, not camera-to-robot registration or execution approval.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from hand_motor_mapping import ticks_to_sim,TICK
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261029/observed_inverse_contact_fit'
CHECK=ROOT/'right_hand_rotation_20261029/inverse_contact_checkpoint'
def mesh_separation_factory(m,cube,geom):
    """Full convex SAT; no remote-margin mesh distance approximation.

    The actual engine contact constraints are independently checked as well.
    https://mujoco.readthedocs.io/en/latest/computation/#geom-distance
    """
    mesh=int(m.geom_dataid[geom]);start=int(m.mesh_vertadr[mesh]);count=int(m.mesh_vertnum[mesh])
    vertices=np.asarray(m.mesh_vert[start:start+count],dtype=float);hull=ConvexHull(vertices)
    points=vertices[hull.vertices]
    normals=hull.equations[:,:3]
    edges=set()
    for f in hull.simplices:
        for a,b in ((f[0],f[1]),(f[1],f[2]),(f[2],f[0])):edges.add(tuple(sorted((int(a),int(b)))))
    directions=np.asarray([vertices[b]-vertices[a] for a,b in edges])
    def unique_axes(a):
        length=np.linalg.norm(a,axis=1);a=a[length>1e-9]/length[length>1e-9,None]
        return np.unique(np.round(a,8),axis=0)
    normals=unique_axes(normals);directions=unique_axes(directions)
    def distance(d):
        A=d.geom_xmat[cube].reshape(3,3);B=d.geom_xmat[geom].reshape(3,3)
        nn=normals@B.T;ee=directions@B.T
        crossed=np.cross(A.T[:,None,:],ee[None,:,:]).reshape(-1,3)
        length=np.linalg.norm(crossed,axis=1);crossed=crossed[length>1e-9]/length[length>1e-9,None]
        axes=np.vstack([A.T,nn,crossed]);radius=np.abs(axes@A)@m.geom_size[cube,:3]
        v=points@B.T+(d.geom_xpos[geom]-d.geom_xpos[cube]);projections=v@axes.T
        return float(np.max(np.maximum(projections.min(0)-radius,-radius-projections.max(0))))
    return distance
def box_separation(m,d,a,b):
    # mj_geomDistance does not supply reliable box-box penetration/fromto
    # information for this model. Use all15 OBB separating axes instead.
    A=d.geom_xmat[a].reshape(3,3);B=d.geom_xmat[b].reshape(3,3)
    delta=d.geom_xpos[a]-d.geom_xpos[b];axes=[*A.T,*B.T]
    axes += [np.cross(x,y) for x in A.T for y in B.T]
    values=[]
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-9:continue
        axis=axis/length
        values.append(abs(delta@axis)-np.sum(m.geom_size[a,:3]*abs(A.T@axis))-np.sum(m.geom_size[b,:3]*abs(B.T@axis)))
    return max(values)
def main():
    OUT.mkdir(exist_ok=True)
    old=OUT/'review.json';archive=OUT/'v1_invalid_box_distance_review.json'
    if old.exists() and not archive.exists():archive.write_bytes(old.read_bytes())
    m=mujoco.MjModel.from_binary_path(str(CHECK/'checkpoint_model.mjb'))
    seed=json.loads((ROOT/'right_hand_rotation_20261029/gapped_inverse_cube/cube_seed.json').read_bytes())['cube_qpos']
    initial=np.asarray(seed[:3])*1000;rotation0=Rotation.from_quat(np.asarray(seed[3:])[[1,2,3,0]]).as_matrix()
    up=-m.opt.gravity/np.linalg.norm(m.opt.gravity);cube=m.geom('cube').id
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    active=[g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in hand and ((int(m.geom_contype[cube])&int(m.geom_conaffinity[g])) or (int(m.geom_contype[g])&int(m.geom_conaffinity[cube])))]
    mesh_distance={g:mesh_separation_factory(m,cube,g) for g in active if int(m.geom_type[g])==7}
    tips=[m.geom(n).id for n in ('mf_tip','th_tip')];bases=[m.geom(n).id for n in ('palm_collision_1','if_bs_collision_1')]
    reports=[]
    for folder,angle in (('gapped_inverse_trial_20261029_184252_272166',-53.66281841871696),('supported_inverse_trial_20261029_231345_341471',-32.376021450567855)):
        source=ROOT/folder/'result.json';r=json.loads(source.read_bytes());samples=[s for s in r['samples'] if s['phase']=='stage_8_horizontal_hold'][-20:]
        actual=np.median([s['actual'] for s in samples],axis=0);goal=np.asarray(samples[-1]['goal'])
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=ticks_to_sim(goal)
        rot=Rotation.from_rotvec(up*np.radians(angle)).as_matrix()@rotation0;q=Rotation.from_matrix(rot).as_quat();d.qpos[-4:]=q[[3,0,1,2]]
        def dist(g):
            if int(m.geom_type[cube])==6 and int(m.geom_type[g])==6:return box_separation(m,d,cube,int(g))*1000
            if g in mesh_distance:return mesh_distance[g](d)*1000
            return float(mujoco.mj_geomDistance(m,d,cube,int(g),.2,None))*1000
        def evaluate(center,detail=False):
            d.qpos[-7:-4]=center/1000.;mujoco.mj_forward(m,d)
            distances=np.array([dist(g) for g in active]);tip_dist=np.array([dist(g) for g in tips]);base_dist=min(dist(g) for g in bases)
            actual_pen=np.zeros(len(active));lookup={g:i for i,g in enumerate(active)}
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube in (a,b):
                    other=b if a==cube else a
                    if other in lookup:actual_pen[lookup[other]]=max(actual_pen[lookup[other]],max(0.,-float(c.dist)*1000))
            residual=np.r_[tip_dist,base_dist,np.minimum(distances+.002,0)*20,actual_pen*20,(center-initial)*.01]
            if detail:
                su,sp=inspect(d)
                return dict(center_mm=center.tolist(),tip_distances_mm=tip_dist.tolist(),closest_base_mm=base_dist,
                    max_cube_penetration_mm=max(float(max(actual_pen)),max(0.,-float(min(distances)))),max_self_mm=max(sp.values(),default=0.),max_support_mm=max(su.values(),default=0.))
            return residual
        seeds=[initial+np.array([x,y,z]) for x,y,z in ((0,0,0),(-10,0,0),(10,0,0),(0,-10,0),(0,10,0),(-15,-10,5),(-15,10,5),(10,0,10))]
        solutions=[]
        for center in seeds:
            fit=least_squares(evaluate,center,bounds=(initial-30,initial+30),max_nfev=200,xtol=1e-8,ftol=1e-8,gtol=1e-8)
            solutions.append((float(np.linalg.norm(fit.fun)),fit.x))
        score,center=min(solutions,key=lambda x:x[0]);detail=evaluate(center,True)
        fitted=bool(max(abs(np.asarray(detail['tip_distances_mm'])))<=.5 and abs(detail['closest_base_mm'])<=.5 and detail['max_cube_penetration_mm']<=.05 and detail['max_self_mm']<=1e-6 and detail['max_support_mm']<=.15)
        item=dict(source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),observed_yaw_deg=angle,
            actual_median_ticks=actual.tolist(),command_ticks=goal.tolist(),assumed_initial_cube_rotation_not_registered=True,
            conditional_two_tip_and_base_fit=fitted,score=score,**detail,hardware_execution_allowed=False,devices_opened=False)
        reports.append(item);print(json.dumps(item),flush=True)
        if fitted:
            path=OUT/(folder+'_conditional_state_v3.npz');np.savez_compressed(path,actual=actual,goal=goal,cube_qpos=d.qpos[-7:].copy())
        (OUT/'review.json').write_text(json.dumps(dict(cases=reports,geometry_unchanged=True,box_box_distance_uses_OBB_SAT=True,
            actual_mujoco_contact_penetration_also_checked=True,v1_box_distance_fit_rejected=True,convex_mesh_SAT_also_checked=True,
            conditional_not_physical_registration=True,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
