"""Free-cube model with explicit fixed-goal pauses; no hardware access."""
import sys
import screen_loaded_yaw_path_20261027 as screen
from adaptive_yaw_loaded_plan_20261029 import ROOT,OUT,STRESS_PATH
if __name__=='__main__':
    sys.argv=['loaded_stress','--candidate',STRESS_PATH.relative_to(ROOT).as_posix(),'--output-folder',(OUT/'loaded_pause_stress_27').relative_to(ROOT).as_posix()]
    screen.main()
