"""Lower/retreat the recorded thumb before raising index/middle roots."""
import numpy as np
from build_taught_palm_transfer_20261118 import interpolate

ROOT_COLUMNS=[7,11]  # physical motor IDs8 and12

def reordered_pitch(old):
    phases=old['phases'];goals=old['goals'];start=old['start']
    def part(name):
        indices=np.flatnonzero(phases==name)
        if not len(indices):raise RuntimeError('Pitch source phase missing: '+name)
        return goals[indices].copy()
    first=part('stage_6_loaded_quarter_reference')
    clearance=part('stage_8a_loaded_thumb_lift_clearance')
    retreat=part('stage_9_loaded_thumb_retreat')
    for rows in (first,clearance,retreat):rows[:,ROOT_COLUMNS]=start[ROOT_COLUMNS]
    target=goals[np.flatnonzero(phases=='stage_9_loaded_thumb_retreat')[-1]].copy()
    lift=interpolate(retreat[-1],target)
    end=int(np.flatnonzero(phases=='stage_9_loaded_thumb_retreat')[-1])+1
    rows=np.vstack([first,clearance,retreat,lift,goals[end:]])
    labels=np.concatenate([np.array(['thumb_lower_first']*len(first)),
        np.array(['thumb_clear_first']*len(clearance)),np.array(['thumb_retreat_first']*len(retreat)),
        np.array(['index_middle_raise']*len(lift)),np.array(['source_'+str(p) for p in phases[end:]])])
    if not np.array_equal(rows[-1],goals[-1]):raise RuntimeError('Reordered pitch end changed')
    review=dict(source_goals=len(goals),queued_goals=len(rows),root_motor_ids=[8,12],
        root_fixed_until_index=len(first)+len(clearance)+len(retreat),
        root_raise_goals=len(lift),thumb_before_root_raise=True,source_tail_start=end,
        same_final_goal=True,source_file_unchanged=True)
    return rows,labels,review
