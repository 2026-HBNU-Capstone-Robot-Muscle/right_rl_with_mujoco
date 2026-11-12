"""Track extra motion of a held joint without confusing prior lag with drift.

Each joint's reference is the fresh measured position immediately before its
last commanded change. While a joint is held, only the interval from that
position to its current goal, plus 20 ticks at each end, is permitted. The
separate absolute tracking/current/voltage checks remain mandatory.
"""
import numpy as np


def _raw(value):
    a=np.asarray(value)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Invalid held-joint raw state')
    return a.astype(np.int64)


class HeldJointFault(RuntimeError):
    def __init__(self,details):
        self.details=details
        first=details['joints'][0]
        super().__init__('A fixed finger joint left its held-position bounds: '
            f"ID{first['id']} actual={first['actual']}, reference={first['reference']}, "
            f"goal={first['goal']}, allowed=[{first['lower']},{first['upper']}]; no target advance")


class HeldJointGuard:
    VERSION='last_command_settling_interval_v1'
    def __init__(self,position,command):
        self.reference=_raw(position);self.command=_raw(command)
    def check(self,position,command,indices,phase):
        actual=_raw(position);goal=_raw(command)
        if not np.array_equal(goal,self.command):raise ValueError('Held-joint command changed outside scheduled update')
        indices=np.asarray(indices,dtype=int)
        if np.any(indices<0) or np.any(indices>=16):raise ValueError('Invalid held-joint IDs')
        low=np.minimum(self.reference,goal)-20;high=np.maximum(self.reference,goal)+20
        bad=[int(i) for i in indices if not low[i]<=actual[i]<=high[i]]
        if bad:
            raise HeldJointFault(dict(version=self.VERSION,phase=phase,margin_ticks=20,
                joints=[dict(id=i+1,actual=int(actual[i]),reference=int(self.reference[i]),goal=int(goal[i]),
                    lower=int(low[i]),upper=int(high[i]),tracking_error_ticks=int(actual[i]-goal[i])) for i in bad]))
    def advance(self,position,old_command,new_command):
        actual=_raw(position);old=_raw(old_command);new=_raw(new_command)
        if not np.array_equal(old,self.command):raise ValueError('Held-joint update is out of sequence')
        changed=new!=old
        self.reference[changed]=actual[changed]
        self.command=new.copy()
