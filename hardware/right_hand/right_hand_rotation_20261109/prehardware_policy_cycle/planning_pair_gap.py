"""Bounded separating-axis cost for planning; not a collision certificate.

Any positive interval gap on a unit axis is a separation lower bound. A small
set of axes can miss separation, so nonpositive values do not prove contact.
Native contact guards and final exact sweep certificates remain independent.
"""
import numpy as np
from convex_pair_gap import ConvexPairGap


class PlanningPairGap:
    def __init__(self,model,geoms,max_face_axes=24):
        geometry=ConvexPairGap(model,geoms);self.cache={}
        for geom,(vertices,normals,edges) in geometry.cache.items():
            if len(normals)>max_face_axes:
                normals=normals[np.linspace(0,len(normals)-1,max_face_axes).astype(int)]
            self.cache[geom]=(vertices,normals)

    def gap(self,data,first,second):
        a,na=self.cache[first];b,nb=self.cache[second]
        A=data.geom_xmat[first].reshape(3,3);B=data.geom_xmat[second].reshape(3,3)
        delta=data.geom_xpos[second]-data.geom_xpos[first]
        axes=np.vstack([na@A.T,nb@B.T,np.eye(3),delta.reshape(1,3)])
        lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-12]/lengths[lengths>1e-12,None]
        x=(a@A.T)@axes.T;y=(b@B.T+delta)@axes.T
        return float(np.max(np.maximum(y.min(0)-x.max(0),x.min(0)-y.max(0))))
