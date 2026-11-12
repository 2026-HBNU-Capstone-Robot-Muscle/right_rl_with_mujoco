"""Check the archived evaluator against a fresh, equivalent native parent run."""
from pathlib import Path
import hashlib
import json
import numpy as np

OUT=Path(__file__).resolve().parent


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_evaluator_revision(expected_historical_sha256):
    parent=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_whole_hand_v1_active_ccd50'
    fresh=OUT/'next_cycle/staged_model_nominal_integer_encoders_admissible_correction_extra_hold400_gain010_projection_whole_hand_v1_active_ccd50'
    if sha(parent/'source_evaluate_staged_shadow_model.py')!=expected_historical_sha256:
        raise AssertionError('Historical evaluator snapshot mismatch')
    if sha(fresh/'source_evaluate_staged_shadow_model.py')!=sha(OUT/'evaluate_staged_shadow_model.py'):
        raise AssertionError('Current evaluator has no matching fresh native comparison')
    before=json.loads((parent/'review.json').read_text());after=json.loads((fresh/'review.json').read_text())
    if before['model_provenance']['model_sha256']!=after['model_provenance']['model_sha256']:
        raise AssertionError('Parent comparison changed the model')
    if before['steps']!=4118 or after['steps']!=4118 or before['stop']!=after['stop']:
        raise AssertionError('Parent stop differs')
    a=np.load(parent/'research_trace.npz',allow_pickle=False)['goals'];b=np.load(fresh/'research_trace.npz',allow_pickle=False)['goals']
    if not np.array_equal(a,b):raise AssertionError('Executed parent goals differ')
    a=np.load(parent/'end_state.npz',allow_pickle=False);b=np.load(fresh/'end_state.npz',allow_pickle=False)
    for key in ('qpos','qvel','ctrl','act','qacc_warmstart'):
        if not np.array_equal(a[key],b[key]):raise AssertionError('Native parent endpoint differs: '+key)
    return dict(historical_source_sha256=expected_historical_sha256,
        current_source_sha256=sha(OUT/'evaluate_staged_shadow_model.py'),
        revalidation_path=(fresh/'review.json').relative_to(OUT).as_posix(),
        revalidation_sha256=sha(fresh/'review.json'),identical_goals_and_bound_state=True,
        scope='The existing4118-goal parent rollout through its recorded stop; not whole-cycle certification')
