"""Pure trajectory proposals for the user's distal-first cube transfer.

No serial, camera, torque or learning APIs. Callers supply reviewed native bounds
and must separately check dynamics and current physical state before execution.
"""
from dataclasses import dataclass
import numpy as np

TIP_IDS=(1,9)
NEXT_IDS=(2,11)
RING_NEXT_ID=15

@dataclass(frozen=True)
class TransferProposal:
    start: np.ndarray
    goals: np.ndarray
    phases: tuple
    targets: dict

def _ticks(value,name):
    a=np.asarray(value,dtype=float)
    if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError(name+' must contain16 finite integer motor ticks in ID1..16 order')
    return a.astype(np.int64)

def rearward_side_offset(front_normal,side_normal,distance_mm):
    """Move away from the white/front edge, tangentially along the contact side.

    The caller must register these directions; a photograph alone does not
    establish a world-frame distance or certify this contact interpretation.
    """
    front=np.array(front_normal,dtype=float,copy=True);side=np.array(side_normal,dtype=float,copy=True)
    if (front.shape!=(3,) or side.shape!=(3,) or not np.isfinite(front).all()
            or not np.isfinite(side).all() or np.linalg.norm(front)<1e-12 or np.linalg.norm(side)<1e-12
            or not np.isfinite(distance_mm) or distance_mm<0):raise ValueError('Invalid contact-frame directions or distance')
    front/=np.linalg.norm(front);side/=np.linalg.norm(side)
    tangent=-front+side*float(front@side)
    if np.linalg.norm(tangent)<1e-9:raise ValueError('Front direction must not be parallel to contact-side normal')
    return tangent/np.linalg.norm(tangent)*(distance_mm/1000.)

def propose(start,lower,upper,*,fraction=1.,hold_goals=54):
    """Bend TH1/MF9 first, then TH2/MF11, within supplied native limits.

    fraction controls a research sweep toward the supplied upper bounds. It is
    not a hardware acceptance rule or permission to enlarge those bounds.
    """
    q=_ticks(start,'start');lo=np.ceil(np.asarray(lower,dtype=float));hi=np.floor(np.asarray(upper,dtype=float))
    if (lo.shape!=(16,) or hi.shape!=(16,) or not np.isfinite(lo).all() or not np.isfinite(hi).all()
            or np.any(lo>hi) or np.any(q<lo) or np.any(q>hi)):raise ValueError('Invalid native interval or start')
    if not np.isfinite(fraction) or not 0<fraction<=1:raise ValueError('fraction must lie in (0,1]')
    if isinstance(hold_goals,bool) or not isinstance(hold_goals,int) or hold_goals<0:raise ValueError('Invalid hold count')
    previous=q.copy();rows=[];phases=[];targets={}
    for phase,ids in (('tips_first',TIP_IDS),('next_links_second',NEXT_IDS)):
        ix=np.asarray(ids)-1;target=previous.copy();target[ix]=np.rint(previous[ix]+fraction*(hi[ix]-previous[ix])).astype(int)
        n=max(1,int(np.ceil(np.max(abs(target-previous))/2)))
        for k in range(1,n+1):
            rows.append(np.rint(previous+(target-previous)*k/n).astype(int));phases.append(phase)
        previous=target;targets[phase]={str(i):int(target[i-1]) for i in ids}
    for _ in range(hold_goals):rows.append(previous.copy());phases.append('transfer_hold')
    goals=np.asarray(rows,dtype=np.int64)
    if (np.any(goals<lo) or np.any(goals>hi) or np.max(abs(np.diff(np.vstack([q,goals]),axis=0)))>2):
        raise RuntimeError('Internal command bound violation')
    return TransferProposal(q,goals,tuple(phases),targets)

def cap_ring_next_for_candidate(goals,maximum_ticks):
    """Candidate-specific guard against repeating the observed ID15 overreach.

    This does not change a motor range or any current/tracking stop threshold.
    Path spacing and complete dynamics must be checked again after applying it.
    """
    a=np.asarray(goals)
    if a.ndim!=2 or a.shape[1]!=16 or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
        raise ValueError('Expected Nx16 integer candidate goals')
    if not np.isfinite(maximum_ticks) or maximum_ticks!=int(maximum_ticks):raise ValueError('Integer cap required')
    result=a.copy();result[:,RING_NEXT_ID-1]=np.minimum(result[:,RING_NEXT_ID-1],int(maximum_ticks))
    return result
