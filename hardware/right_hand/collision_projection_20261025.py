"""Offline configuration projection. Changes proposed goals, never geometry."""
import numpy as np
import mujoco
from scipy.optimize import minimize
from hand_motor_mapping import sim_to_ticks,TICK

class CollisionProjection:
    def __init__(self,model,margin_m=.0005,guard_pairs=()):
        self.m=model;self.d=mujoco.MjData(model);self.margin=margin_m
        self.guard_pairs=[tuple(sorted((model.geom(a).id,model.geom(b).id))) for a,b in guard_pairs]
        self.hand={model.body('palm').id}
        for b in range(model.nbody):
            if int(model.body_parentid[b]) in self.hand:self.hand.add(b)
        self.support={model.geom(n).id for n in ('support_book','support_glass','support_wood')}
        self.lo=np.maximum(model.jnt_range[:16,0],model.actuator_ctrlrange[:,0])
        self.hi=np.minimum(model.jnt_range[:16,1],model.actuator_ctrlrange[:,1])
    def place(self,q):
        self.d.qpos[:16]=q;self.d.qpos[-7:]=[10,10,10,1,0,0,0];self.d.qvel[:]=0
        mujoco.mj_forward(self.m,self.d)
    def pairs(self,q):
        self.place(q);pairs={}
        for c in self.d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(self.m.geom_bodyid[a]) in self.hand,int(self.m.geom_bodyid[b]) in self.hand
            if c.dist<self.margin and (ha and hb or a in self.support and hb or b in self.support and ha):
                key=tuple(sorted((a,b)));pairs[key]=min(float(c.dist),pairs.get(key,1.))
        for a,b in self.guard_pairs:
            distance=float(mujoco.mj_geomDistance(self.m,self.d,a,b,.05,None))
            if distance<self.margin:pairs[a,b]=min(distance,pairs.get((a,b),1.))
        return pairs
    def maximum_overlap(self,q):return max([-v for v in self.pairs(q).values()]+[0.])
    def quantize_goals(self,q):
        """Round NEW proposed targets inside native bounds, without editing data."""
        low=np.ceil((self.lo+np.pi)/TICK-1e-10)
        high=np.floor((self.hi+np.pi)/TICK+1e-10)
        counts=np.clip(np.rint((np.asarray(q)+np.pi)/TICK),low,high)
        return sim_to_ticks(counts*TICK-np.pi)
    def distances(self,q,pairs):
        self.place(q)
        return np.array([mujoco.mj_geomDistance(self.m,self.d,a,b,.05,None) for a,b in pairs])
    def jacobian(self,q,pairs):
        self.place(q);rows=[]
        for a,b in pairs:
            witness=np.zeros(6);dist=mujoco.mj_geomDistance(self.m,self.d,a,b,.05,witness)
            if abs(dist)>=.049 or abs(dist)<1e-10:rows.append(np.zeros(16));continue
            normal=(witness[3:]-witness[:3])/dist
            ja,jb=np.zeros((3,self.m.nv)),np.zeros((3,self.m.nv))
            mujoco.mj_jac(self.m,self.d,ja,None,witness[:3],int(self.m.geom_bodyid[a]))
            mujoco.mj_jac(self.m,self.d,jb,None,witness[3:],int(self.m.geom_bodyid[b]))
            rows.append((normal@(jb-ja))[:16])
        return np.asarray(rows).reshape(-1,16)
    def check_jacobian(self,q):
        pairs=sorted(self.pairs(q));analytic=self.jacobian(q,pairs);numerical=np.zeros_like(analytic);h=1e-6
        for j in range(16):
            delta=np.eye(16)[j]*h
            numerical[:,j]=(self.distances(q+delta,pairs)-self.distances(q-delta,pairs))/(2*h)
        error=float(np.max(abs(analytic-numerical))) if pairs else 0.
        return dict(pair_count=len(pairs),maximum_absolute_error_m_per_rad=error,
            passes_smooth_fixture_check=error<5e-5)
    def project(self,reference,start=None,bound_rad=.45):
        reference=np.asarray(reference,float);start=reference.copy() if start is None else np.asarray(start,float)
        lo=np.maximum(self.lo,reference-bound_rad);hi=np.minimum(self.hi,reference+bound_rad)
        if np.any(lo>hi):raise ValueError('Reference is outside reachable projection bounds')
        x=np.clip(start,lo,hi);pairs=set(self.pairs(reference))|set(self.pairs(x));attempts=[]
        for outer in range(8):
            if not pairs:x=np.clip(reference,lo,hi);break
            ordered=sorted(pairs)
            result=minimize(lambda q:float(np.sum((q-reference)**2)),x,
                jac=lambda q:2*(q-reference),method='SLSQP',bounds=list(zip(lo,hi)),
                constraints=[dict(type='ineq',fun=lambda q:(self.distances(q,ordered)-self.margin)*1000,
                    jac=lambda q:self.jacobian(q,ordered)*1000)],
                options=dict(maxiter=180,ftol=1e-10))
            x=result.x;new=self.pairs(x);pairs.update(new)
            attempts.append(dict(optimizer_success=bool(result.success),maximum_overlap_mm=self.maximum_overlap(x)*1000))
            if self.maximum_overlap(x)<1e-7 and np.all(self.distances(x,sorted(pairs))>=self.margin-1e-7):break
        return x,dict(attempts=attempts,maximum_overlap_mm=self.maximum_overlap(x)*1000,
            maximum_reference_deviation_rad=float(np.max(abs(x-reference))),
            native_ranges_satisfied=bool(np.all(x>=self.lo-1e-9) and np.all(x<=self.hi+1e-9)))
