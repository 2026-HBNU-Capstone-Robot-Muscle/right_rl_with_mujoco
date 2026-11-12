"""Raised EMPTY support before natural placement, then prior full yaw route.
The index-root support stays raised throughout. There is no higher-placement
target for the operator; the cube must freely settle onto the prepared hand.
"""
import argparse,json
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
R=Path(__file__).resolve().parent
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*i/n for i in range(1,n+1)]).astype(int)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--raise-ticks',type=int,choices=(96,160,192,208),required=True);ap.add_argument('--extra-swivel',type=int,choices=(0,128,256),default=0);args=ap.parse_args();O=R/f'right_hand_rotation_20261104/raised_cradle_{args.raise_ticks}_yaw{args.extra_swivel}';O.mkdir(parents=True,exist_ok=True);old=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');start=old['start'].copy();start[[7,11]]+=args.raise_ticks;goal=old['start'].copy();goal[7]=start[7];rows=blend(start,goal).tolist();phases=['stage_5_loaded_grasp']*len(rows);source=old['goals'][:int(old['forward_end_sample'])].copy();source[:,7]=start[7];rows.extend(source);phases.extend(old['phases'][:int(old['forward_end_sample'])].tolist());previous=np.array(rows[-1]);extra=previous.copy();extra[9]+=args.extra_swivel
 if args.extra_swivel:part=blend(previous,extra);rows.extend(part);phases.extend(['stage_7_horizontal_yaw']*len(part))
 rows.extend(np.tile(extra,(67,1)));phases.extend(['stage_8_horizontal_hold']*67);values=np.array(rows);lo,hi=limits();assert np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(np.vstack([start,values]),axis=0)).ravel())<=2;np.savez_compressed(O/'candidate.npz',start=start,goals=values,phases=np.array(phases),forward_end_sample=np.array(len(values)));print(json.dumps(dict(folder=O.relative_to(R).as_posix(),rows=len(values),seconds=len(values)*.075,natural_free_seating_required=True,no_high_placement_target=True,hardware_execution_allowed=False,motor_writes=0)))
if __name__=='__main__':main()
