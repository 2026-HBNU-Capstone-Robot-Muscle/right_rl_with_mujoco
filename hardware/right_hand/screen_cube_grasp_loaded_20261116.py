"""Reuse unchanged loaded force/geometry checker with temporary support footprint."""
import screen_natural_grasp_hold_20261030 as screen
from final_pose_temporary_support_20261116 import build
screen.build=build
if __name__=='__main__':screen.main()
