"""Stop only the explicitly listed, verified offline replay processes."""
import os,signal,json,argparse
from pathlib import Path
R=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('pids',type=int,nargs='+');a=ap.parse_args();result=[]
expected=str(R/'replay_canonical_grip_matrix_20261105.py')
allowed={'weak_refinement/i-96_t-64','weak_refinement/i-77_t-56','weak_refinement/i-96_t-56'}
for pid in a.pids:
    proc=Path('/proc')/str(pid)
    if not proc.exists():result.append(dict(pid=pid,status='already_finished'));continue
    argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    if expected not in argv or '--output-tag' not in argv or argv[argv.index('--output-tag')+1] not in allowed:raise RuntimeError('Refuse to signal an unrecognized process')
    uidline=next(v for v in (proc/'status').read_text().splitlines() if v.startswith('Uid:'))
    if int(uidline.split()[1])!=os.getuid():raise RuntimeError('Refuse to signal another user process')
    tag=argv[argv.index('--output-tag')+1];os.kill(pid,signal.SIGTERM);result.append(dict(pid=pid,tag=tag,status='own_offline_replay_terminated_for_redundant_certificate_recalculation'))
out=R/'right_hand_rotation_20261105/canonical_body_grip/weak_refinement/aborted_duplicate_certificate_runs.json'
out.write_text(json.dumps(dict(runs=result,no_completed_result_claimed=True,physical_devices_used=False),indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
