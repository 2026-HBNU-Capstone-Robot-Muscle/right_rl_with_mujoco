"""Reuse unchanged empty-case criteria with the additional pitch setup path."""
import screen_calibrated_empty_margin_20261028 as screen
from pitch_handoff_plan_20261028 import PitchHandoffPlan

screen.OfflineRecovery=PitchHandoffPlan

if __name__=='__main__':screen.main()
