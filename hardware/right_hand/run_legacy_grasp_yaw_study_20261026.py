"""Dependency-ordered offline regrasp/yaw experiment in one WSL process."""
import argparse,json,sys
from pathlib import Path
import plan_thumb_opposed_pinch_transition_20261026 as planner
import replay_thumb_opposed_pinch_transition_20261026 as replay
import adaptive_after_flip_pinch_20261026 as adaptive
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--folder',required=True);args=parser.parse_args();folder=(ROOT/args.folder).resolve();folder.relative_to(ROOT);common=['--pair','3','--seed-folder',args.folder]
    print('STUDY',args.folder,'path screen',flush=True);sys.argv=['planner']+common;planner.main();path=json.loads((folder/'rrt_thumb_pair3_review.json').read_bytes())
    if not path['solution_found']:return
    print('STUDY free cube regrasp',flush=True);sys.argv=['replay']+common;replay.main();grip=json.loads((folder/'replay_thumb_pair3_review.json').read_bytes())
    if not grip['promising_regrasp']:return
    for direction in (1,-1):
        print('STUDY yaw direction',direction,flush=True);sys.argv=['adaptive']+common+['--thumb-pair','--warm-end','--direction',str(direction)];adaptive.main()
if __name__=='__main__':main()
