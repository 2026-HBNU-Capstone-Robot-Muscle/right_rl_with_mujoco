"""NumPy-only MF clearance proposals from encoder values; no device interface.

Positive separating-axis values prove static separation of this one pair.
They are not a dynamic or whole-hand collision certificate. Original native
guards remain independent in every model trial.
"""
import numpy as np
from shadow_runtime import matrix,quaternion,TICK,SIM_IDS


def rotation(axis,theta):
    axis=np.asarray(axis,float);axis=axis/np.linalg.norm(axis);x,y,z=axis
    skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.eye(3)+np.sin(theta)*skew+(1.-np.cos(theta))*(skew@skew)


class FingerClearance:
    def __init__(self,path):
        self.z=dict(np.load(path,allow_pickle=False))

    def poses(self,q):
        q=np.asarray(q,float)
        if q.shape!=(16,) or not np.isfinite(q).all():raise ValueError('Finite sim-order16 encoder angles required')
        z=self.z;poses={0:(np.eye(3),np.zeros(3))}
        for body in z['body_order']:
            body=int(body);parent=int(z['body_parentid'][body]);parent_R,parent_t=poses[parent]
            R=parent_R@matrix(quaternion(z['body_quat'][body]));t=parent_t+parent_R@z['body_pos'][body]
            for joint in range(int(z['body_jntadr'][body]),int(z['body_jntadr'][body]+z['body_jntnum'][body])):
                if int(z['jnt_type'][joint])!=3:raise ValueError('Only the declared fixed/hinge ancestor chain is supported')
                adr=int(z['jnt_qposadr'][joint]);anchor=t+R@z['jnt_pos'][joint];axis=R@z['jnt_axis'][joint];delta=rotation(axis,q[adr]-z['qpos0'][adr]);t=anchor+delta@(t-anchor);R=delta@R
            poses[body]=(R,t)
        result=[]
        for i,body in enumerate(z['geom_bodyid']):
            R,t=poses[int(body)];result.append((R@matrix(quaternion(z['geom_quat'][i])),t+R@z['geom_pos'][i]))
        return result

    def gap_mm(self,q):
        (A,ta),(B,tb)=self.poses(q);z=self.z;delta=tb-ta
        axes=np.vstack([z['normals0']@A.T,z['normals1']@B.T,np.eye(3),delta.reshape(1,3)])
        lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-12]/lengths[lengths>1e-12,None]
        a=(z['vertices0']@A.T)@axes.T;b=(z['vertices1']@B.T+delta)@axes.T
        return float(np.max(np.maximum(b.min(0)-a.max(0),a.min(0)-b.max(0)))*1000)

    def select(self,measured_ticks,commanded_ticks,requested_ticks,lower,upper,margin_mm=.15):
        measured=np.asarray(measured_ticks,float);before=np.asarray(commanded_ticks,float);requested=np.asarray(requested_ticks,float)
        if any(x.shape!=(16,) or not np.isfinite(x).all() for x in (measured,before,requested)):raise ValueError('Finite motor-order16 inputs required')
        if not np.array_equal(before,np.rint(before)) or not np.array_equal(requested,np.rint(requested)):raise ValueError('Integer acknowledged and requested targets required')
        if np.any(before<lower) or np.any(before>upper) or np.any(requested<lower) or np.any(requested>upper):raise ValueError('Original native target bounds are required')
        nominal=before+np.clip(requested-before,-2,2);qm=(measured[SIM_IDS-1]-2048)*TICK
        # The common ancestor cancels most root motion. The encoder-dependent
        # proposal still includes all MF controls when projecting the next pose.
        def projected(command):return qm+(command-before)[SIM_IDS-1]*TICK
        initial=self.gap_mm(projected(nominal))
        if initial>=margin_mm:return nominal.astype(int),dict(applied=False,projected_gap_mm=initial,margin_mm=margin_mm)
        candidates=[]
        for pip in (-2,-1,0,1,2):
            for dip in (-2,-1,0,1,2):
                candidate=nominal.copy();candidate[10]=before[10]+pip;candidate[8]=before[8]+dip
                if np.any(candidate<lower) or np.any(candidate>upper):continue
                gap=self.gap_mm(projected(candidate));error=float(np.sum((candidate-nominal)**2))
                score=(gap<margin_mm,max(0.,margin_mm-gap),error) if gap<margin_mm else (False,0.,error)
                candidates.append((score,candidate,gap))
        if not candidates:raise ValueError('No bounded MF clearance proposal')
        _,selected,gap=min(candidates,key=lambda row:row[0]);assert np.max(abs(selected-before))<=2
        return selected.astype(int),dict(applied=not np.array_equal(selected,nominal),projected_gap_mm=gap,margin_mm=margin_mm,static_projected_margin_satisfied=gap>=margin_mm,not_a_dynamic_collision_certificate=True)
