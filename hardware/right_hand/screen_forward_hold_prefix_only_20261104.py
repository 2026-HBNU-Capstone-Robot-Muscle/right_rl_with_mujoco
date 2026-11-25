"""Current EMPTY proof only; no devices and no loaded admission."""
import os,sys
os.environ['JAX_PLATFORMS']='cpu'
from pathlib import Path
from forward_hold_plan_20261104 import ForwardHoldPlan,REQUEST,OUT
import screen_calibrated_empty_margin_20261028 as s
if __name__=='__main__':
    s.OfflineRecovery=ForwardHoldPlan;sys.argv=['empty','--request',str(REQUEST),'--output-folder',(OUT/'empty_recovery_screen').relative_to(Path(__file__).resolve().parent).as_posix()];s.main()
