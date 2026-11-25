"""Conservative separating-plane certificates for the complete layer sweep.

The 90-degree sweep is enclosed by a cylinder. Separating each existing convex
hand geom from that cylinder proves clearance at the queried simulated pose.
A circumscribed polygonal prism is used only to find candidate plane normals;
every returned certificate is rechecked against the exact cylinder support.
No model, dynamics, collision mask or physical device is modified.
"""
import numpy as np
from scipy.spatial import ConvexHull
from upper_layer_cylinder_clearance_20261105 import RADIUS,LOWER,UPPER,precompute

MID=(LOWER+UPPER)/2
HALF=(UPPER-LOWER)/2

def cylinder_minimum(normal):
    n=np.asarray(normal,float)
    return n[...,1]*MID-np.abs(n[...,1])*HALF-RADIUS*np.linalg.norm(n[...,[0,2]],axis=-1)

def normalized_axes(axes):
    a=np.asarray(axes,float).reshape(-1,3);length=np.linalg.norm(a,axis=1);a=a[length>1e-10]/length[length>1e-10,None]
    # Both orientations are considered by the interval test below.
    sign=np.where(np.abs(a[:,0])>1e-8,np.sign(a[:,0]),np.where(np.abs(a[:,1])>1e-8,np.sign(a[:,1]),np.sign(a[:,2])))
    return np.unique(np.round(a*sign[:,None],9),axis=0)

def find_plane(vertices,edges,face_normals,minimum=.3,sides=64):
    """Return unit normal and offset with the cylinder outside geom n.v<=b."""
    v=np.asarray(vertices,float);theta=(np.arange(sides)+.5)*2*np.pi/sides;r=RADIUS/np.cos(np.pi/sides)
    circle=np.c_[r*np.cos(theta),r*np.sin(theta)]
    prism=np.vstack([np.c_[circle[:,0],np.full(sides,y),circle[:,1]] for y in (LOWER,UPPER)])
    pedges=np.vstack([[0,1,0],np.c_[np.roll(circle[:,0],-1)-circle[:,0],np.zeros(sides),np.roll(circle[:,1],-1)-circle[:,1]]])
    gedges=v[edges[:,1]]-v[edges[:,0]]
    pnorm=np.vstack([[0,1,0],np.c_[np.cos(np.arange(sides)*2*np.pi/sides),np.zeros(sides),np.sin(np.arange(sides)*2*np.pi/sides)]])
    axes=normalized_axes(np.vstack([face_normals,pnorm,np.cross(gedges[:,None,:],pedges[None,:,:]).reshape(-1,3)]));best=None;bestgap=-float('inf')
    for start in range(0,len(axes),512):
        a=axes[start:start+512];va=v@a.T;pa=prism@a.T;forward=pa.min(axis=0)-va.max(axis=0);reverse=va.min(axis=0)-pa.max(axis=0);values=np.maximum(forward,reverse);j=int(np.argmax(values))
        if values[j]>bestgap:
            bestgap=float(values[j]);n=a[j] if forward[j]>=reverse[j] else -a[j];n=n/np.linalg.norm(n);b=float(max(v@n));gap=float(cylinder_minimum(n)-b);best=(n,b,gap)
    return best if best is not None and best[2]>minimum else None

class SweepCertificate:
    def __init__(self,m,hand,minimum=.3):
        self.minimum=minimum;self.geometry=precompute(m,hand);self.faces=[];self.face_b=[]
        for g,v,e in self.geometry:
            hull=ConvexHull(v);eq=np.unique(np.round(hull.equations,10),axis=0);length=np.linalg.norm(eq[:,:3],axis=1);normals=eq[:,:3]/length[:,None];self.faces.append(normals);self.face_b.append(np.max(v@normals.T,axis=0)*1000+1e-6)
        self.ids=np.array([g for g,v,e in self.geometry],int);self.normal=np.zeros((len(self.ids),3));self.offset=np.zeros(len(self.ids));self.ready=np.zeros(len(self.ids),bool);self.repairs=0;self.sat_searches=0;self.queries=0

    def _repair(self,index,d,C,B):
        g,v,e=self.geometry[index];G=d.geom_xmat[g].reshape(3,3);p=(d.geom_xpos[g]-C)@B*1000;rot=G.T@B;axes=self.faces[index]@rot;b=self.face_b[index]+axes@p;gap=cylinder_minimum(axes)-b;j=int(np.argmax(gap))
        if gap[j]>self.minimum:
            self.normal[index]=self.faces[index][j];self.offset[index]=self.face_b[index][j];self.ready[index]=True;self.repairs+=1;return
        vc=v@rot*1000+p;self.sat_searches+=1;plane=find_plane(vc,e,axes,self.minimum)
        if plane is None:
            self.normal[index]=self.faces[index][j];self.offset[index]=self.face_b[index][j];self.ready[index]=True;return
        n,_,_=plane;local=rot@n;local/=np.linalg.norm(local);self.normal[index]=local;self.offset[index]=float(max(v@local))*1000+1e-6;self.ready[index]=True;self.repairs+=1

    def query(self,d,C,B):
        self.queries+=1
        for i in np.flatnonzero(~self.ready):self._repair(int(i),d,C,B)
        G=d.geom_xmat[self.ids].reshape(-1,3,3);world=np.einsum('nij,nj->ni',G,self.normal);axes=world@B;p=(d.geom_xpos[self.ids]-C)@B*1000;values=cylinder_minimum(axes)-self.offset-np.einsum('ni,ni->n',axes,p)
        failed=np.flatnonzero(values<=self.minimum)
        for i in failed:self._repair(int(i),d,C,B)
        if len(failed):
            world=np.einsum('nij,nj->ni',G,self.normal);axes=world@B;values=cylinder_minimum(axes)-self.offset-np.einsum('ni,ni->n',axes,p)
        return float(min(values)),[int(i) for i in np.flatnonzero(values<=self.minimum)]

    def metadata(self):
        return dict(geoms=len(self.ids),minimum_required_gap_mm=self.minimum,queries=self.queries,plane_repairs=self.repairs,polygon_searches=self.sat_searches,exact_cylinder_support_rechecked=True,model_changes=0,certificate_scope='Queried simulated poses only; continuous physical motion and geometry calibration are not certified')
