"""Reuse the saved-data analysis for the second completed observation."""
from pathlib import Path
import review_forward_hold_actual_20261105 as review
R=Path(__file__).resolve().parent
if __name__=='__main__':
    review.F=R/'forward_hold_observe_20261105_040030_646453'
    review.OUT=R/'right_hand_rotation_20261105/preintegration/repeat/actual_review_040030'
    review.main()
