"""Create inspectable standalone copies; do not modify the earlier proofs/code."""
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def replace(s,old,new,count=1):
    if s.count(old)!=count:raise ValueError('Unexpected source block count: '+old[:90])
    return s.replace(old,new,count)
def main():
    s=(ROOT/'screen_loaded_quarter_finish_20261025.py').read_text(encoding='utf-8')
    s=replace(s,'from loaded_quarter_finish_plan_20261025 import LoadedQuarterFinishPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals',
        'from recovered_forward_plan_20261026 import RecoveredForwardPlan as BookGraspPlan,TURN_PHASE,END_HOLD,FINISH_PHASE,loaded_basis,finish_goals\nfrom interrupted_thumb_contact_policy_20261026 import metadata as interrupted_contact_policy')
    s=replace(s,'plan=plan_type(request);raw=plan.source_position()',
        "plan=plan_type(request);raw=plan.source_position()\n    contact_scope=interrupted_contact_policy(raw)\n    THUMB_NEIGHBOR_LIMIT_MM=contact_scope['initial_pair_limit_mm']\n    SELF_PAIRS=globals()['SELF_PAIRS']+tuple(contact_scope['additional_initial_pairs'])")
    s=replace(s,"raise ValueError('OFF initial casing/ring contact exceeds its separately recorded model bound')",
        "raise ValueError('OFF initial contact bound exceeded: '+str({p:dict(depth_mm=self_pairs.get(p,0.),limit_mm=bound) for p,bound in ((CASING_PAIR,CASING_INITIAL_LIMIT_MM),(EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM),(THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM)) if self_pairs.get(p,0.)>bound}))",count=2)
    s=replace(s,'r=dict(plan_version=plan.PLAN_VERSION,',
        'r=dict(recorded_interrupted_contact_policy=interrupted_contact_policy(raw),plan_version=plan.PLAN_VERSION,')
    (ROOT/'screen_recovered_forward_20261026.py').write_text(s,encoding='utf-8')
    g=(ROOT/'loaded_quarter_finish_gate_20261025.py').read_text(encoding='utf-8')
    g=replace(g,"checker=ROOT/'screen_loaded_quarter_finish_20261025.py'","checker=ROOT/'screen_recovered_forward_20261026.py'")
    g=replace(g,'from loaded_quarter_finish_plan_20261025 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals',
        'from recovered_forward_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals\nfrom interrupted_thumb_contact_policy_20261026 import metadata as interrupted_contact_policy')
    g=replace(g,"r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)",
        "r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)\n    contact_scope=interrupted_contact_policy(raw)\n    THUMB_NEIGHBOR_LIMIT_MM=contact_scope['initial_pair_limit_mm']\n    SELF_PAIRS=globals()['SELF_PAIRS']+tuple(contact_scope['additional_initial_pairs'])\n    if r.get('recorded_interrupted_contact_policy')!=contact_scope:\n        raise RuntimeError('Recorded OFF-contact scope/proof mismatch; no writes')")
    (ROOT/'recovered_forward_gate_20261026.py').write_text(g,encoding='utf-8')
    print('Created standalone scoped recovery checker and validator; earlier files unchanged.')
if __name__=='__main__':main()
