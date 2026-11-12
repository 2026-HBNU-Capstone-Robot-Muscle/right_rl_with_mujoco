"""Photo prefix uses unchanged hand goals and an explicit initial model resolution."""
import hashlib,json
from loaded_quarter_trial_plan_20261025 import LoadedQuarterTrialPlan,ROOT,extra_hashes as parent_hashes
from support_photo_contact_resolution_20261025 import metadata,PAIRS,NUMERICAL_ZERO_MM
FIXTURE=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025/current_pose_v3'
FILES=('support_photo_path_plan_20261025.py','screen_support_photo_path_20261025.py','leap_quarter_support_photo_20261025.py','support_photo_camera_20261025.py','support_photo_contact_resolution_20261025.py')
def extra_hashes():return {**parent_hashes(),**{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}}
class SupportPhotoPathPlan(LoadedQuarterTrialPlan):
    PLAN_VERSION='quarter_support_photo_initial_thumb_resolution_v3_20261025'
    proof_hashes=staticmethod(extra_hashes)
def validate_proof(plan,path):
    from loaded_quarter_trial_gate_20261025 import validate_proof as original
    r=original(plan,path)
    if r.get('photo_initial_thumb_contact_resolution')!=metadata() or r.get('initial_resolution_forces_preserved') is not True:
        raise RuntimeError('Photo mode initial-contact resolution metadata mismatch; no writes')
    rows=r.get('initial_resolution_discarded_maximums_by_front')
    if not isinstance(rows,list) or len(rows)!=3:raise RuntimeError('Photo contact-resolution evidence missing; no writes')
    for values in rows:
        if not isinstance(values,dict) or not set(values).issubset(PAIRS) or any(not isinstance(v,(int,float)) or not 0<=v<=NUMERICAL_ZERO_MM for v in values.values()):
            raise RuntimeError('Photo contact-resolution scope/depth mismatch; no writes')
    return r
def run_screen(request,output,**kwargs):
    from thumb_live_start_gate import run_screen as core
    from loaded_quarter_trial_gate_20261025 import screen_failure_details
    return core(request,output,checker=ROOT/'screen_support_photo_path_20261025.py',failure_details=screen_failure_details,**kwargs)
