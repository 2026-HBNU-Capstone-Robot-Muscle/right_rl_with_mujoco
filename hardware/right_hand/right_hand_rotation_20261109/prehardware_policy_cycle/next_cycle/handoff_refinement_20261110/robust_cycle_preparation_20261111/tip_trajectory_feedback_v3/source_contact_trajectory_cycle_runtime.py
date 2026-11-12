"""Prefer MF clearance adjustments that disturb the tip trajectory least.

This is a soft proposal cost; persistent MF contact is not required. All actual
physics guards and whole-hand support checks remain independent.
"""
import numpy as np
from shadow_runtime import SIM_IDS,TICK
from cycle_joint_clearance import FingerClearance
from whole_cycle_shadow_runtime import WholeCycleShadowController


class TipTrajectoryClearance(FingerClearance):
    def select(self,measured_ticks,commanded_ticks,requested_ticks,lower,upper,margin_mm=.04):
        measured=np.asarray(measured_ticks,float);before=np.asarray(commanded_ticks,float);requested=np.asarray(requested_ticks,float)
        if any(x.shape!=(16,) or not np.isfinite(x).all() for x in (measured,before,requested)):raise ValueError('Finite motor-order16 vectors required')
        if np.any(requested<lower) or np.any(requested>upper):raise ValueError('Requested native goal range exceeded')
        nominal=before+np.clip(requested-before,-2,2);qm=(measured[SIM_IDS-1]-2048)*TICK
        def project(command):return qm+(command-before)[SIM_IDS-1]*TICK
        gap=self.gap_mm(project(nominal))
        if gap>=margin_mm:return nominal.astype(int),dict(applied=False,projected_gap_mm=gap,margin_mm=margin_mm)
        R0,t0=self.poses(project(nominal))[1];candidates=[]
        for pip in (-2,-1,0,1,2):
            for dip in (-2,-1,0,1,2):
                command=nominal.copy();command[10]=before[10]+pip;command[8]=before[8]+dip
                if np.any(command<lower) or np.any(command>upper):continue
                q=project(command);value=self.gap_mm(q);R,t=self.poses(q)[1];shift=float(np.linalg.norm(t-t0)*1000)
                cost=50*shift**2+.01*float(np.sum((command-nominal)**2))+.1*float(np.sum((R-R0)**2))
                candidates.append(((value<margin_mm,max(0.,margin_mm-value),cost),command,value,shift))
        if not candidates:raise ValueError('No bounded MF trajectory-clearance candidate')
        _,goal,value,shift=min(candidates,key=lambda row:row[0]);return goal.astype(int),dict(applied=not np.array_equal(goal,nominal),projected_gap_mm=value,margin_mm=margin_mm,tip_shift_from_requested_mm=shift,soft_tip_trajectory_cost_not_contact_invariant=True,not_a_dynamic_collision_certificate=True)


class TrajectoryClearanceCycleController(WholeCycleShadowController):
    def __init__(self,bundle,observation_mode='registered_observer'):
        super().__init__(bundle,observation_mode);self.clearance=TipTrajectoryClearance(self.folder/'kinematics.npz')
