"""Offline complete hand-route replay of the measured dynamic turn candidate."""
import argparse
from screen_book_grasp_trial import main
from book_dynamic_yaw_plan import BookDynamicYawPlan

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output')
    a=parser.parse_args();raise SystemExit(main(a.request,a.output,plan_type=BookDynamicYawPlan))
