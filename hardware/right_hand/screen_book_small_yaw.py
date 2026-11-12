"""Device-free whole hand-route replay; the cube is excluded from this gate."""
import argparse
from screen_book_grasp_trial import main
from book_small_yaw_plan import BookSmallYawPlan

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('request');parser.add_argument('output')
    args=parser.parse_args()
    raise SystemExit(main(args.request,args.output,plan_type=BookSmallYawPlan))
