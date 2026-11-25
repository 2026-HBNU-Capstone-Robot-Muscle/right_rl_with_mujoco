"""Native response stress holds at the two observed camera-failure regions."""
from book_empty_preparation_plan import LIFT
from loaded_quarter_trial_plan_20261025 import RETURN_PHASE
from empty_camera_gap_hold_20261026 import metadata as gap_metadata,MAX_SINGLE_HOLD_S
def expected_holds(rows):
    n_lift=sum(p==LIFT for p,q in rows);n_return=sum(p==RETURN_PHASE for p,q in rows)
    return [dict(phase=p,goal_index=min(index,n),seconds=MAX_SINGLE_HOLD_S)
            for p,index,n in ((LIFT,91,n_lift),(RETURN_PHASE,55,n_return)) if n]
def add_stress_holds(dynamics,rows):
    holds=expected_holds(rows);by_phase={v['phase']:v for v in holds};seen={};out=[]
    for phase,goal,duration in dynamics:
        out.append((phase,goal,duration));seen[phase]=seen.get(phase,0)+1
        e=by_phase.get(phase)
        if e and seen[phase]==e['goal_index']:out.append((phase,goal,e['seconds']))
    return out,holds
