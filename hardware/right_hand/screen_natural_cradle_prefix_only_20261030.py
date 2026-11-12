"""Prepare initial EMPTY evidence independently of still-running loaded tests."""
import sys
from natural_cradle_grasp_plan_20261030 import ROOT,OUT,REQUEST,NaturalCradleGraspPlan
def main():
 import screen_calibrated_empty_margin_20261028 as s
 s.OfflineRecovery=NaturalCradleGraspPlan;old=sys.argv
 try:sys.argv=['empty','--request',str(REQUEST),'--output-folder',(OUT/'empty_recovery_screen').relative_to(ROOT).as_posix()];s.main()
 finally:sys.argv=old
if __name__=='__main__':main()
