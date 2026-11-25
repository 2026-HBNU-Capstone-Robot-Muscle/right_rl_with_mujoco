"""Pure EMPTY opening path; never restores a closed grasp before placement."""
import numpy as np

THUMB_NATIVE_RECOVERY_FLOOR=1856
THUMB_CLEARANCE_TICKS=128


def interpolate(rows,start,end,phase):
    n=max(20,int(np.ceil(np.max(abs(end-start))/2)))
    for k in range(1,n+1):rows.append((phase,np.rint(start+(end-start)*k/n).astype(np.int64)))
    return end.copy()


def empty_opening(raw,opened,*,ring_tip_extension_ticks=0):
    a=np.asarray(raw,dtype=np.int64).copy();opened=np.asarray(opened,dtype=np.int64)
    if type(ring_tip_extension_ticks) is not int or not 0<=ring_tip_extension_ticks<=64:
        raise ValueError('Ring-tip release extension must be 0..64 ticks')
    rows=[];release=a.copy()
    # Ring PIP decreases to separate the tip from the palm. While it does,
    # only a small thumb-root range re-entry may occur. In particular the
    # blocked middle root ID12 keeps its fresh measured goal, not 2613.
    release[14]=min(a[14],opened[14])
    release[3]=max(a[3],THUMB_NATIVE_RECOVERY_FLOOR)
    # Optional EMPTY-only separation: extend the tip with the PIP instead of
    # relying on the PIP alone. Native ranges and every exact path are still
    # independently screened. Existing callers retain their original goals.
    release[12]-=ring_tip_extension_ticks
    a=interpolate(rows,a,release,'stage_0_empty_recover')
    thumb=a.copy();delta=opened[:4]-a[:4]
    alpha=min(1.,THUMB_CLEARANCE_TICKS/max(1,int(max(abs(delta)))))
    thumb[:4]=np.rint(a[:4]+delta*alpha).astype(np.int64)
    a=interpolate(rows,a,thumb,'stage_1_thumb_clearance')
    interpolate(rows,a,opened,'stage_2_empty_open')
    return rows
