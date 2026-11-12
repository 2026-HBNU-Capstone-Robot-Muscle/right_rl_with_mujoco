"""Confirm that finite-variation support did not alter the unperturbed replay."""
from pathlib import Path
import subprocess
import sys
import json
import hashlib
import numpy as np
OUT=Path(__file__).resolve().parent;BASE=OUT/'next_cycle/handoff_refinement_20261110'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    tag='unperturbed_cycle_prefix_regression';folder=BASE/tag
    subprocess.run([sys.executable,'-B',str(OUT/'replay_low_friction_feedback_chain.py'),'--tag',tag,'--complete-75ms','--stop-after-stage','MF_first_arrival'],check=True)
    expected=BASE/'robust_mu025_complete_cycle75ms_replay_v39/trajectory.npz';r=json.loads((folder/'review.json').read_text());actual=np.load(folder/'trajectory.npz',allow_pickle=False)['qpos'];original=np.load(expected,allow_pickle=False)['qpos'][:len(actual)];delta=float(np.max(abs(actual-original)));assert r['steps']==3669 and not r['stop'] and delta==0,delta
    result=dict(status='passed',steps=r['steps'],all_qpos_array_values_exactly_equal=True,maximum_qpos_difference=delta,reference=expected.relative_to(OUT).as_posix(),reference_sha256=sha(expected),variation_parameter_support_does_not_change_unperturbed_dynamics=True,devices_opened=0,hardware_execution_allowed=False)
    (folder/'regression.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)

if __name__=='__main__':main()
