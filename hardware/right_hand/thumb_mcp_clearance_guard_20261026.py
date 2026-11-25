"""Use the validated release position instead of an arbitrary20tick goal error."""
import time
import numpy as np
from book_grasp_trial_plan import RING_RELEASE
from loaded_quarter_reentry_20261025 import cmc_native_min

def policy(plan,proof):
    raw=plan.source_position();limits=[];evidence=[]
    for case in proof['static_screens']:
        if not any('th_' in pair for pair in case['initial_self_contacts_mm']):continue
        start=raw+np.asarray(case['offset_ticks'],dtype=int);rows=plan.plan(start)
        ring=[q for phase,q in rows if phase==RING_RELEASE]
        at=case['self_escape_complete_at']
        if (not isinstance(at,int) or not 0<=at<=len(ring)
                or any(ring[k][1]>ring[k-1][1] for k in range(1,len(ring)))):
            raise RuntimeError('Thumb contact does not clear during the validated opening segment; no next target')
        clear=start if at==0 else rows[at-1][1]
        if int(clear[1])>int(start[1]):raise RuntimeError('Thumb clearance direction is not opening; no next target')
        limits.append(int(clear[1]));evidence.append(dict(offset_ticks=case['offset_ticks'],clear_at=at,mcp_ticks=int(clear[1])))
    return dict(revision='measured_certified_thumb_release_v1_20261026',
        threshold_ticks=min(limits)-4 if limits else None,margin_ticks=4,
        thumb_contact_cases=len(evidence),case_clearance_evidence=evidence,
        no_initial_thumb_contact_in_any_case=not bool(limits),
        maximum_snapshot_age_s=.2,goal_following_20tick_stop_removed=True,
        measured_health_tracking_stop_rad=.18)

def verify(state,command,release_policy,report,*,read_at,now=time.monotonic):
    if state is None or not 0<=now()-read_at<=.2:
        raise RuntimeError('No fresh measured thumb opening; no next target')
    if not np.array_equal(state['goal'],command):
        raise RuntimeError('Thumb release snapshot does not match the last accepted goal; no next target')
    limit=release_policy['threshold_ticks'];actual=int(state['position'][1])
    if limit is not None and actual>limit:
        raise RuntimeError(f'Thumb ID2 has not reached the validated release position: actual={actual}, need<={limit}; no next target')
    report.update(thumb_mcp_release_measured_verified=True,thumb_mcp_release_actual_ticks=actual,
        thumb_mcp_release_limit_ticks=limit,thumb_mcp_release_goal_error_ticks=actual-int(command[1]),
        thumb_mcp_release_policy=release_policy)
