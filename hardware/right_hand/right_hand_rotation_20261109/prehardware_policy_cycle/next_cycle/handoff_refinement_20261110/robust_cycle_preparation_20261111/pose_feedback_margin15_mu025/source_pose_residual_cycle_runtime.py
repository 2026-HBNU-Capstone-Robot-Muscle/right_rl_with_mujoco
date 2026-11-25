"""Encoder/object-pose residual using offline identified local responses.

Uses only timestamped cube pose and encoder inputs online, never simulated
contact forces. This is an identified feedback controller, not a new SAC actor.
"""
import numpy as np
from shadow_runtime import quaternion,multiply,SIM_IDS,TICK
from whole_cycle_shadow_runtime import WholeCycleShadowController


class PoseResidualCycleController(WholeCycleShadowController):
    def __init__(self,bundle,observation_mode='registered_observer'):
        super().__init__(bundle,observation_mode)
        r=np.load(self.folder/'response.npz',allow_pickle=False);self.response_phases=r['phases'];self.response_gains=r['gains'];self.orientation_weight=float(r['orientation_weight']);self.bias=np.zeros(16)
        if self.response_gains.shape!=(len(self.response_phases),16,6) or not np.isfinite(self.response_gains).all():raise ValueError('Finite16x6 pose response matrices required')

    def propose(self,sample):
        result=super().propose(sample)
        if result['state']=='warming' or not 1700<=self.phase<=2420:return result
        reference=self.ref_qpos[self.phase];q=quaternion(sample['cube_quaternion']);qr=quaternion(reference[-4:]);delta=quaternion(multiply(qr,q*np.array([1,-1,-1,-1])))
        if delta[0]<0:delta=-delta
        length=np.linalg.norm(delta[1:]);angle=2*np.arctan2(length,delta[0]);rot=np.zeros(3) if length<1e-12 else delta[1:]/length*angle*180/np.pi
        position=(reference[-7:-4]-np.asarray(sample['cube_position']))*1000
        error=np.r_[position,rot*self.orientation_weight]
        index=int(np.argmin(abs(self.response_phases-self.phase)));gain=self.response_gains[index]
        if np.linalg.norm(position)<.2 and np.linalg.norm(rot)<.2:target=np.zeros(16)
        else:target=np.clip(.35*(gain@error),-32,32)
        bias=.8*self.bias+.2*target
        desired=np.clip(np.rint(self.pending['baseline']+bias),np.ceil(self.low),np.floor(self.high));before=np.asarray(sample['commanded_ticks'])
        goal,shield=self.clearance.select(sample['measured_ticks'],before,desired,self.low,self.high,self.meta['MF_static_margin_mm'])
        if np.max(abs(goal-before))>2:raise ValueError('Pose residual exceeded unchanged two-tick execution bound')
        self.pending['bias']=bias;self.pending['goal']=goal.copy();result['goal_ticks']=goal.tolist();result['details'].update(pose_feedback_applied=True,pose_error_mm_deg=error.tolist(),absolute_residual_ticks=bias.tolist(),local_response_phase=int(self.response_phases[index]),clearance=shield,source='saved_teacher_plus_identified_pose_residual_not_SAC')
        return result

    def commit_shadow(self,sequence,goal_ticks):
        bias=self.pending.get('bias',self.bias.copy()) if self.pending is not None else self.bias.copy()
        super().commit_shadow(sequence,goal_ticks);self.bias=bias
