"""Existing recovery criteria applied to the new combined queue's empty prefix."""
import screen_calibrated_empty_margin_20261028 as screen
from pitch_slow_yaw_plan_20261029 import PitchSlowYawPlan
screen.OfflineRecovery=PitchSlowYawPlan
if __name__=='__main__':screen.main()
