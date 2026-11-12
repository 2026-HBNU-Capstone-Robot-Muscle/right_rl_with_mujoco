"""Same goals as the placement comparison, with a scoped OFF-contact proof."""
import hashlib
from forward_cube_placement_20261026 import ForwardPlacementPlan,ROOT
from loaded_quarter_finish_plan_20261025 import extra_hashes as parent_hashes,TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals
from quarter_finish_empty_plan_20261025 import INITIAL_SELF_SCOPE
FILES=('interrupted_thumb_contact_policy_20261026.py','recovered_forward_plan_20261026.py',
    'recovered_forward_gate_20261026.py','screen_recovered_forward_20261026.py',
    'leap_recovered_forward_placement_20261026.py','leap_recovered_forward_placement.cmd',
    'forward_cube_placement_20261026.py')
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/recovered_forward_fixture'
def extra_hashes():return {**parent_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
class RecoveredForwardPlan(ForwardPlacementPlan):
    PLAN_VERSION='same_turn_recorded_interrupted_OFF_contact_v1_20261026'
    FOLDER_PREFIX='recovered_forward_placement'
    STATUS='recovered_forward_placement_complete'
    TITLE='RECOVERED PLACEMENT: EMPTY contact separation -> PLACE5mm toward tips -> SAME turn -> REMOVE -> OFF.'
    proof_hashes=staticmethod(extra_hashes)
