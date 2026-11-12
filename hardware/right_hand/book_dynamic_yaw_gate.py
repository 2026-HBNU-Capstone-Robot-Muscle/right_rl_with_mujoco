"""Full native hand-route proof bound to the revised measured dynamic candidate."""
from book_grasp_trial_gate import validate_proof,screen_failure_details
from thumb_live_start_gate import run_screen as run_native_screen
from book_grasp_trial_plan import ROOT


def run_screen(request,output,**kwargs):
    return run_native_screen(request,output,checker=ROOT/'screen_book_dynamic_yaw.py',
        failure_details=screen_failure_details,**kwargs)
