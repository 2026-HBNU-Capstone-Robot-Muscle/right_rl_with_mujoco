"""Independent separating-axis gap for convex hand geometry; no model changes."""
import numpy as np


def signed_axis_gap(a,b,normals_a,normals_b,edges_a,edges_b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    cross=np.cross(np.asarray(edges_a)[:,None,:],np.asarray(edges_b)[None,:,:]).reshape(-1,3)
    lengths=np.linalg.norm(cross,axis=1);cross=cross[lengths>1e-10]/lengths[lengths>1e-10,None]
    axes=np.vstack([normals_a,normals_b,cross]);lengths=np.linalg.norm(axes,axis=1)
    axes=axes[lengths>1e-10]/lengths[lengths>1e-10,None]
    if not len(axes) or not all(np.isfinite(value).all() for value in (a,b,axes)):
        raise ValueError('Finite convex vertices and axes required')
    # Remove common translation before projections to preserve small gap precision.
    center=(a.mean(0)+b.mean(0))/2
    x=(a-center)@axes.T;y=(b-center)@axes.T
    return float(np.max(np.maximum(y.min(0)-x.max(0),x.min(0)-y.max(0))))


def directions(vectors):
    vectors=np.asarray(vectors,float)
    lengths=np.linalg.norm(vectors,axis=1);vectors=vectors[lengths>1e-10]/lengths[lengths>1e-10,None]
    for row in vectors:
        first=np.flatnonzero(abs(row)>1e-10)
        if len(first) and row[first[0]]<0:row*=-1
    return np.unique(np.round(vectors,12),axis=0)


class ConvexPairGap:
    def __init__(self,model,geoms):
        from scipy.spatial import ConvexHull
        self.cache={}
        for geom in geoms:
            kind=int(model.geom_type[geom])
            if kind==7:
                mesh=int(model.geom_dataid[geom]);start=int(model.mesh_vertadr[mesh])
                vertices=np.asarray(model.mesh_vert[start:start+int(model.mesh_vertnum[mesh])],float)
            elif kind==6:
                vertices=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*model.geom_size[geom]
            else:raise ValueError('Only existing convex meshes and boxes are supported')
            hull=ConvexHull(vertices);vertices=vertices[hull.vertices];hull=ConvexHull(vertices)
            edges=set()
            for triangle in hull.simplices:
                for a,b in ((triangle[0],triangle[1]),(triangle[1],triangle[2]),(triangle[2],triangle[0])):
                    edges.add(tuple(sorted((int(a),int(b)))))
            self.cache[geom]=(vertices,directions(hull.equations[:,:3]),
                              directions([vertices[b]-vertices[a] for a,b in edges]))

    def gap(self,data,first,second):
        a,na,ea=self.cache[first];b,nb,eb=self.cache[second]
        A=data.geom_xmat[first].reshape(3,3);B=data.geom_xmat[second].reshape(3,3)
        return signed_axis_gap(a@A.T+data.geom_xpos[first],b@B.T+data.geom_xpos[second],
                               na@A.T,nb@B.T,ea@A.T,eb@B.T)
