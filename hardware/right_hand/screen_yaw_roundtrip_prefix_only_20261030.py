"""Run current EMPTY recovery independently while loaded cases finish."""
import sys
import screen_calibrated_empty_margin_20261028 as screen
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan,ROOT,OUT,REQUEST
screen.OfflineRecovery=YawRoundtripPlan
if __name__=='__main__':
    sys.argv=['empty','--request',str(REQUEST),'--output-folder',(OUT/'empty_recovery_screen').relative_to(ROOT).as_posix()]
    screen.main()
