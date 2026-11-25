"""Promote the exact accepted v3 replay without altering or reissuing its contents."""
import shutil
from recovered_forward_plan_20261026 import FIXTURE,RecoveredForwardPlan
from recovered_forward_gate_20261026 import validate_proof
def main():
    source=FIXTURE/'screen_v3.json';validate_proof(RecoveredForwardPlan(FIXTURE/'request.json'),source)
    dest=FIXTURE/'screen.json';rejected=FIXTURE/'screen_rejected_missing_initial_palm_pair.json'
    if dest.exists() and dest.read_bytes()!=source.read_bytes():
        if rejected.exists() and rejected.read_bytes()!=dest.read_bytes():raise ValueError('Refuse to replace a different rejected replay')
        if not rejected.exists():shutil.copyfile(dest,rejected)
    shutil.copyfile(source,dest)
    if dest.read_bytes()!=source.read_bytes():raise ValueError('Replay promotion changed contents')
    print('Exact accepted replay published. Earlier rejected replay retained.')
if __name__=='__main__':main()
