"""Operator-started, separately screened EMPTY preparation with the book."""
from book_empty_preparation_plan import BookPreparationPlan
from book_empty_preparation_gate import run_screen,validate_proof
from leap_thumb_prepare_current_empty import Bus as CurrentBus,main
from marker_trial_camera import MarkerTrialCamera


class BookCamera(MarkerTrialCamera):
    EXPOSURE=-5.
    EMPTY_ONLY=True


class BookBus(CurrentBus):
    def approve(self,plan,proof):
        self.proof=validate_proof(plan,proof)
        self.plan_module=plan;self.planner=plan.validate_start
        self.screen_approved=True


BookPreparationPlan.CAMERA_TYPE=BookCamera

if __name__=='__main__':
    print('Keep the current 20mm book and fixed base in place. EMPTY HAND for the entire run.',flush=True)
    print('Experimental support separation; photo-based geometry is approximate. No cube or SAC.',flush=True)
    raise SystemExit(main(BookPreparationPlan,bus_type=BookBus,screener=run_screen))
