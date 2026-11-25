"""Reuse one IK proposal; compare support-first dynamic approach orders."""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from audit_opposed_working_grip_20261117 import MODEL,replay,sha
from research_rotation_body_hold_20261104 import restore,motor

BASE=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
PARENT=BASE/'regrasp_middle_inset_index_thumb_middle_backmiddle'


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--order',choices=('thumb_middle_index','together','middle_thumb_index'),required=True)
    a=ap.parse_args()
    prior=json.loads((PARENT/'review.json').read_bytes())
    if not prior['geometry_passed'] or not prior['allocation']:raise RuntimeError('Valid static proposal required')
    source=ROOT/prior['source_path']
    if sha(source)!=prior['source_sha256'] or sha(MODEL)!=prior['model_sha256']:raise RuntimeError('Source/model changed')
    out=BASE/f'regrasp_supportfirst_{a.order}';out.mkdir(exist_ok=False)
    (out/'source_script.py').write_bytes(Path(__file__).read_bytes())
    z=dict(np.load(source,allow_pickle=False));old=np.load(PARENT/'research_goals.npz',allow_pickle=False)
    start=motor(z['ctrl']);target=old['goals'][-1];previous=start.copy();rows=[];phases=[]
    groupmap={'thumb':np.arange(4),'middle':np.arange(8,12),'index':np.arange(4,8),'together':np.arange(16)}
    order=a.order.split('_')
    for phase in order:
        group=groupmap[phase];endpoint=previous.copy();endpoint[group]=target[group]
        count=max(1,int(np.ceil(max(abs(endpoint-previous))/2)))
        rows.extend(np.rint(previous+(endpoint-previous)*j/count).astype(int) for j in range(1,count+1))
        phases.extend([phase]*count);previous=endpoint
    hold_start=len(rows);rows.extend(target.copy() for _ in range(400));phases.extend(['hold']*400)
    m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()});d=restore(m,z)
    cube=m.geom('cube').id;C=d.geom_xpos[cube].copy();B=d.geom_xmat[cube].reshape(3,3).copy()
    began=time.perf_counter();result=replay(m,z,rows,hold_start,out,C,B,required_roles=('mf_tip','th_tip'))
    np.savez_compressed(out/'research_goals.npz',start=start,goals=rows,phases=phases,hold_start=hold_start)
    report=dict(status='support_first_regrasp_order_comparison',order=a.order,
        source_path=source.relative_to(ROOT).as_posix(),source_sha256=sha(source),model_sha256=sha(MODEL),
        static_review_sha256=sha(PARENT/'review.json'),target_sha256=sha(PARENT/'research_goals.npz'),
        script_sha256=sha(Path(__file__)),goals=len(rows),hold_start=hold_start,
        regrasp_and_hold_passed=result['passed'],audit=result,devices_opened=False,
        cube_reset_after_start=False,hardware_execution_allowed=False,elapsed_seconds=time.perf_counter()-began)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('timeline','certificate')},ensure_ascii=True),flush=True)


if __name__=='__main__':main()
