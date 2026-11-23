"""Closest surface point on a convex polytope; local coordinates only."""
import numpy as np
from scipy.spatial import ConvexHull


def prepare(vertices, edges):
    v=np.asarray(vertices,float);e=np.asarray(edges,int)
    equations=np.unique(np.round(ConvexHull(v).equations,12),axis=0)
    length=np.linalg.norm(equations[:,:3],axis=1)
    return v,e,equations[:,:3]/length[:,None],equations[:,3]/length


def closest(target, surface):
    v,e,A,b=surface;p=np.asarray(target,float)
    # The nearest boundary point lies on a face interior, edge, or vertex.
    projected=p-(A@p+b)[:,None]*A
    valid=np.all(A@projected.T+b[:,None]<=1e-9,axis=0)
    a=v[e[:,0]];direction=v[e[:,1]]-a
    t=np.clip(np.einsum('ij,ij->i',p-a,direction)/np.maximum(np.einsum('ij,ij->i',direction,direction),1e-30),0,1)
    candidates=np.vstack([projected[valid],a+t[:,None]*direction])
    return candidates[int(np.argmin(np.sum((candidates-p)**2,axis=1)))].copy()


def verify_box_cases():
    v=np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])
    hull=ConvexHull(v)
    e=np.array(sorted({tuple(sorted((int(a),int(b)))) for tri in hull.simplices
                       for a,b in ((tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0]))}))
    surface=prepare(v,e)
    cases=[([2,.25,-.3],[1,.25,-.3]),([2,2,.2],[1,1,.2]),([2,2,2],[1,1,1]),
           ([.9,.1,0],[1,.1,0]),([1,.2,.3],[1,.2,.3])]
    for p,expected in cases:
        if not np.allclose(closest(p,surface),expected,atol=1e-10):raise RuntimeError('Analytic box case failed')
    rng=np.random.default_rng(17)
    for _ in range(100):
        p=rng.uniform(-3,3,3)
        if np.all(abs(p)<=1):p[0]=2
        if not np.allclose(closest(p,surface),np.clip(p,-1,1),atol=1e-10):raise RuntimeError('Box projection failed')
    return dict(analytic_feature_cases=len(cases),random_box_cases=100,passed=True)


if __name__=='__main__':print(verify_box_cases())
