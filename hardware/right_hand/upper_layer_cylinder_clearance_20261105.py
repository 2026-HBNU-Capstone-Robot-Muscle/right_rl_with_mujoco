"""Conservative complete90deg upper-layer sweep clearance, geometry only.

A55mm square swept through90deg fills its circumscribed disk. Each hand
geom's convex hull is clipped by the upper-layer height slab; its projection
must remain outside that disk. This changes no dynamic collision model.
"""
import numpy as np
from scipy.spatial import ConvexHull
RADIUS=27.5*np.sqrt(2)
LOWER,UPPER=-27.5,-55/6

def radial_distance(points):
    p=np.unique(np.asarray(points,float),axis=0)
    if not len(p):return float('inf')
    if len(p)==1:return float(np.linalg.norm(p[0]))
    if len(p)>=3:
        try:
            h=ConvexHull(p)
            if np.all(h.equations[:,-1]<=1e-9):return 0.
            p=p[h.vertices]
        except Exception:
            axis=np.argmax(np.ptp(p,axis=0));p=p[np.argsort(p[:,axis])][[0,-1]]
    a=p;b=np.roll(p,-1,axis=0);e=b-a;den=np.einsum('ij,ij->i',e,e);t=np.clip(-np.einsum('ij,ij->i',a,e)/np.maximum(den,1e-20),0,1)
    return float(np.min(np.linalg.norm(a+t[:,None]*e,axis=1)))

def precompute(m,hand):
    out=[]
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in hand or not(m.geom_contype[g] or m.geom_conaffinity[g]):continue
        kind=int(m.geom_type[g])
        if kind==7:
            mesh=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mesh]);vertices=np.asarray(m.mesh_vert[a:a+int(m.mesh_vertnum[mesh])],float)
        elif kind==6:vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[g]
        else:raise RuntimeError('Unhandled hand geometry '+str(m.geom(g).name))
        h=ConvexHull(vertices);v=vertices[h.vertices];h=ConvexHull(v);edges=sorted({tuple(sorted((int(a),int(b)))) for tri in h.simplices for a,b in ((tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0]))})
        out.append((g,v,np.array(edges,int)))
    return out

def clearance_for_vertices(v,edges,exact=True):
    vertical=max(float(v[:,1].min()-UPPER),float(LOWER-v[:,1].max()))
    if not exact:return max(vertical,radial_distance(v[:,[0,2]])-RADIUS)
    if vertical>1e-8:return vertical
    keep=v[(v[:,1]>=LOWER-1e-8)&(v[:,1]<=UPPER+1e-8)];a=v[edges[:,0]];b=v[edges[:,1]];pieces=[keep]
    for y in (LOWER,UPPER):
        crossing=((a[:,1]-y)*(b[:,1]-y)<0);aa=a[crossing];bb=b[crossing]
        if len(aa):pieces.append(aa+(bb-aa)*((y-aa[:,1])/(bb[:,1]-aa[:,1]))[:,None])
    points=np.vstack(pieces)
    return vertical if not len(points) else radial_distance(points[:,[0,2]])-RADIUS

def gaps(m,d,C,B,geoms,exact=True):
    result=[]
    for g,vertices,edges in geoms:
        world=vertices@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];v=(world-C)@B*1000
        gap=clearance_for_vertices(v,edges,exact)
        result.append((m.geom(g).name,float(gap)))
    return result
