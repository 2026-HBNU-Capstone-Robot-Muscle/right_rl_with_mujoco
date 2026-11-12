"""Independent hand mechanics for the archived nominal complete cycle."""
import argparse
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore,Guard,ticks_to_sim,sha,save_json

p=argparse.ArgumentParser(); p.add_argument('--gain',required=True,type=float,choices=(.5,1.,1.5)); p.add_argument('--rf16-fix',action='store_true'); a=p.parse_args()
source=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle/next_cycle/handoff_refinement_20261110/initial_second_turn_hold_replay/research_goals.npz'
folder=ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation'; folder.mkdir(exist_ok=True)
if a.rf16_fix:
    folder=folder/'rf16_2177_v2';source=folder/'candidate.npz'
output=folder/f'hand_gain{a.gain:g}.json'
if output.exists(): raise RuntimeError('Preserve hand mechanics result')
q=np.load(source,allow_pickle=False)['goals']; m,initial,_,_,provenance=assets('active_ccd50')
m.actuator_gainprm[:,0]*=a.gain; m.actuator_biasprm[:,1]*=a.gain
mask=m.geom_bodyid==m.body('cube').id; m.geom_contype[mask]=0; m.geom_conaffinity[mask]=0
d=restore(m,initial); g=Guard(m); stop=[]
for index,command in enumerate(q):
    d.ctrl[:]=ticks_to_sim(command)
    for _ in range(75):
        mujoco.mj_step(m,d)
        stop=[v for v in g.check(d) if v in ('self_mm','support_mm','tracking_rad','range_rad','nonfinite_state')]
        if stop: break
    if stop: break
report={'status':'nominal_complete_cycle_hand_only_screen','source_sha256':sha(source),'gain':a.gain,
        'goals_done':index+1,'total_goals':len(q),'stop':stop,'passed':not stop and index+1==len(q),
        'worst':g.worst,'model_provenance':provenance,'cube_disabled_only_for_separate_hand_check':True,
        'every_executed_ms_checked':1,'native_thresholds_changed':False,'devices_opened':0,'hardware_allowed':False}
save_json(output,report)
print({k:report[k] for k in ('gain','goals_done','stop','passed','worst')},flush=True)
