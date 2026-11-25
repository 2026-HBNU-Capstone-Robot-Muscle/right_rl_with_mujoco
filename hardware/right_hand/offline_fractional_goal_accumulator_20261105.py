"""Fractional tick planning for OFFLINE optimization; no motor transport.

Rounding each tiny update independently can erase motion forever. Preserve the
fractional planner state, quantize only the recorded goal, and enforce original
integer goal and increment bounds before accepting any update.
"""
import numpy as np

class FractionalGoals:
    def __init__(self, start, lower, upper, step_limit=2):
        self.continuous=np.asarray(start,float).copy();self.lower=np.ceil(np.asarray(lower,float));self.upper=np.floor(np.asarray(upper,float));self.step_limit=int(step_limit)
        if self.continuous.shape!=(16,) or self.lower.shape!=(16,) or self.upper.shape!=(16,):raise ValueError('16 motor entries required')
        if self.step_limit!=2 or not np.isfinite(np.r_[self.continuous,self.lower,self.upper]).all() or np.any(self.lower>self.upper):raise ValueError('invalid offline planning bounds')
        self.goal=np.rint(self.continuous).astype(int)
        if np.any(self.continuous<self.lower) or np.any(self.continuous>self.upper):raise ValueError('initial planner state outside bounds')
    def increment_bounds(self):
        return (np.maximum(self.lower-self.continuous,self.goal-self.step_limit-self.continuous),np.minimum(self.upper-self.continuous,self.goal+self.step_limit-self.continuous))
    def advance(self, increment):
        delta=np.asarray(increment,float)
        if delta.shape!=(16,) or not np.isfinite(delta).all():raise ValueError('finite16-vector required')
        proposed=self.continuous+delta
        if np.any(proposed<self.lower-1e-10) or np.any(proposed>self.upper+1e-10):raise ValueError('native planner bound')
        goal=np.rint(proposed).astype(int)
        if np.max(abs(goal-self.goal))>self.step_limit:raise ValueError('integer goal increment bound')
        self.continuous=proposed;self.goal=goal
        return goal.copy()
