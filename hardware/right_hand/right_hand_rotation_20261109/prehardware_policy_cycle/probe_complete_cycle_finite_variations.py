"""Run bounded finite changes of the new complete model path, without devices."""
import argparse
import sys
import subprocess
import json
from pathlib import Path
OUT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--case',choices=('mu024','mu026','xplus1'),required=True);a=p.parse_args()
    options={'mu024':['--sliding-friction','0.24'],'mu026':['--sliding-friction','0.26'],'xplus1':['--initial-x-mm','1']}[a.case]
    parts=['robust_mu025_feedback_pressure_v12/force1.5','late_teacher_checkpoint_for_contact_floor','robust_mu025_late_TH_normal_retraction_v32','robust_mu025_late_TH_retracted_route_v33','robust_mu025_late_TH_normal_release_v35/force0.8','robust_mu025_final_clear_hold_v37/MF_clear','robust_mu025_final_pose_space_v38']
    command=[sys.executable,'-B',str(OUT/'replay_low_friction_feedback_chain.py'),'--tag','complete_cycle_variation_'+a.case,'--complete-75ms','--verify-final-hold',*options]
    for part in parts:command+=['--append-segment',part]
    subprocess.run(command,check=True)

if __name__=='__main__':main()
