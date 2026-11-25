"""Offline bookkeeping for separating a specified existing EMPTY contact.

Never exempts a new contact pair or a deeper penetration than the initial
pose. Static command geometry must decrease per pair. Dynamics may oscillate
below the initial depths, but must separate by the bounded deadline.
"""
import math


class ContactEscape:
    def __init__(self,initial,allowed,max_initial_mm,deadline,*,monotonic):
        self.initial=dict(initial);self.previous=dict(initial);self.allowed=set(allowed)
        self.deadline=deadline;self.monotonic=monotonic
        self.unapproved=0.;self.increase=0.;self.complete_at=None
        for pair,depth in initial.items():
            if not math.isfinite(depth) or depth<0:raise ValueError('Invalid initial contact depth')
            if pair not in self.allowed or (max_initial_mm is not None and depth>max_initial_mm):
                self.unapproved=max(self.unapproved,depth)
        if not initial:self.complete_at=0

    def observe(self,pairs,at):
        for pair,depth in pairs.items():
            if not math.isfinite(depth) or depth<0:raise ValueError('Invalid contact depth')
            permitted=self.initial.get(pair,0.)
            if pair not in self.initial or depth>permitted+1e-6:
                self.unapproved=max(self.unapproved,depth)
            increase=depth-self.previous.get(pair,0.)
            self.increase=max(self.increase,increase)
            if self.monotonic and increase>1e-6:self.unapproved=max(self.unapproved,depth)
            if at>self.deadline and depth>1e-6:self.unapproved=max(self.unapproved,depth)
        if not pairs and self.complete_at is None:self.complete_at=at
        self.previous=dict(pairs)

    def result(self):
        return dict(initial_self_contacts_mm=self.initial,
            self_escape_max_unapproved_overlap_mm=self.unapproved,
            self_escape_max_step_increase_mm=self.increase,
            self_escape_complete_at=self.complete_at,
            self_escape_complete=bool(self.complete_at is not None and self.complete_at<=self.deadline))
