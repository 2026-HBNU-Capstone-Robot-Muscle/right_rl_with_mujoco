"""Same physical goal path; precise read-paced voltage confirmation only."""
import hashlib
from loaded_thumb_rotation_plan_20261026 import *
from loaded_thumb_rotation_plan_20261026 import LoadedThumbRotationPlan,extra_hashes as previous_hashes
FIXTURE=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026/loaded_repeat_fixture_v2_precise_support_resolution'
FILES=('loaded_repeat_plan_20261026.py','loaded_repeat_voltage_20261026.py','loaded_repeat_gate_20261026.py',
       'screen_loaded_repeat_20261026.py','leap_loaded_repeat_20261026.py','leap_loaded_thumb_rotation_retry.cmd',
       'repeat_support_model_resolution_20261026.py')
def extra_hashes():return {**previous_hashes(),**{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}}
class LoadedRepeatPlan(LoadedThumbRotationPlan):
    PLAN_VERSION='loaded_same_goals_precise_read_paced_voltage_v2_support_resolution_20261026'
    PRECISE_READ_PACED_VOLTAGE=True
    FOLDER_PREFIX='loaded_repeat';STATUS='loaded_repeat_complete'
    TITLE='LOADED REPEAT: same EMPTY preparation -> PLACE -> same full turn -> REMOVE -> OFF.'
    proof_hashes=staticmethod(extra_hashes)
