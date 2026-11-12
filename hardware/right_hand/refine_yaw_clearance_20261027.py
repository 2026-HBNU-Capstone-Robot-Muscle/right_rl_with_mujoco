"""Regenerate diagnostic integer goals with more clearance, no device API."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--thumb-relax-ticks',type=int,default=8);parser.add_argument('--preempt-middle-contact',action='store_true');parser.add_argument('--early-root-backoff-ticks',type=int,default=8);args=parser.parse_args()
    if not 0<=args.thumb_relax_ticks<=16:raise ValueError('Diagnostic thumb relaxation bound')
    source=ROOT/'right_hand_rotation_20261027/pair_yaw_finish_root_offsets/candidate.npz';z=np.load(source);out=ROOT/f'right_hand_rotation_20261027/yaw90_clearance_t{args.thumb_relax_ticks}';out=out.with_name(out.name+'_early_root'+(str(args.early_root_backoff_ticks) if args.early_root_backoff_ticks!=8 else '')) if args.preempt_middle_contact else out;out.mkdir(exist_ok=True);lo,hi=limits();previous=z['start'].copy();goals=[];angles=[];changed=0
    for i,(original,angle) in enumerate(zip(z['goals'],z['desired_angles']),1):
        q=original.copy();q[10]=min(int(q[10]),3270);q[0]-=int(np.rint(min(1.,max(0.,(i-406)/20))*args.thumb_relax_ticks))
        if args.preempt_middle_contact:
            if 1086<i<=1146:q[11]-=int(np.rint(min(1.,(i-1086)/40)*args.early_root_backoff_ticks))
            elif 1146<i<1162:q[11]-=int(np.rint((1162-i)/16*args.early_root_backoff_ticks))
        changed+=int(np.any(q!=original));count=max(1,int(np.ceil(max(abs(q-previous))/2)))
        goals.extend(np.rint(previous+(q-previous)*k/count).astype(int) for k in range(1,count+1));angles.extend([angle]*count);previous=q
    values=np.vstack([z['start'],goals]);assert np.isfinite(values).all() and np.all(values>=lo) and np.all(values<=hi) and max(abs(np.diff(values,axis=0)).ravel())<=2
    np.savez_compressed(out/'candidate.npz',start=z['start'],goals=np.array(goals),desired_angles=np.array(angles),bias_ticks=z['bias_ticks'])
    report=dict(status='conservative_clearance_path_regeneration',source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),thumb_relax_ticks=args.thumb_relax_ticks,middle_pip_max_goal_ticks=3270,preempt_middle_contact=args.preempt_middle_contact,early_root_backoff_ticks=args.early_root_backoff_ticks,changed_original_samples=changed,goals=len(goals),goal_step_ticks=2,native_goal_bounds_verified=True,physical_limits_or_gains_changed=False,free_cube_replay_required=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'regeneration.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report))
if __name__=='__main__':main()
