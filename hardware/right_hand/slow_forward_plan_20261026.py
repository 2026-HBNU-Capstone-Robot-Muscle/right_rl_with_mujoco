"""Same recorded goals at75ms cycles; no reduction of voltage protection."""
import hashlib
from recovered_forward_plan_20261026 import RecoveredForwardPlan,ROOT,extra_hashes as parent_hashes,TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals
from slow_forward_protocol_20261026 import SlowForwardCapture
FILES=('slow_forward_plan_20261026.py','slow_forward_protocol_20261026.py','slow_forward_gate_20261026.py',
    'screen_slow_forward_20261026.py','slow_forward_current_runtime_20261026.py',
    'leap_slow_forward_placement_20261026.py','leap_slow_forward_placement.cmd','slow_thumb_seam_resolution_20261026.py')
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/slow_forward_fixture_v2_seam_resolution'
def extra_hashes():return {**parent_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
class SlowForwardPlan(RecoveredForwardPlan):
    PLAN_VERSION='same_goals_75ms_voltage_floor_unchanged_v1_20261026'
    SAMPLE_PERIOD=.075
    ON_TIMEOUT=180
    FOLDER_PREFIX='slow_forward_placement'
    STATUS='slow_forward_placement_complete'
    TITLE='SLOW PLACEMENT: EMPTY recovery -> PLACE -> same turn at1.5x duration -> REMOVE -> OFF.'
    CAPTURE_PROTOCOL_TYPE=SlowForwardCapture
    proof_hashes=staticmethod(extra_hashes)
    def plan(self,raw):
        rows=super().plan(raw)
        if (len(rows)+40)*.085+2>self.ON_TIMEOUT:
            raise ValueError('Slow route exceeds the bounded180s motion allowance')
        return rows
