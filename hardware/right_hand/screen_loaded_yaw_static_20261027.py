"""Unloaded commanded-geometry screen of a loaded trajectory, no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts
from book_empty_preparation_plan import FRONTS,offsets
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--path',required=True);parser.add_argument('--output-folder',required=True);parser.add_argument('--all-offsets',action='store_true');args=parser.parse_args();out=ROOT/args.output_folder;out.resolve().relative_to(ROOT);out.mkdir(parents=True,exist_ok=True);source=ROOT/args.path;source.resolve().relative_to(ROOT);data=np.load(source);goals=data['goals'];cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);before=time.perf_counter();reports=[]
    for front in FRONTS:
        m,_=build(env,out/f'scene_{front:g}.xml',book_projection_mm=front);correct_model(m,np.array(json.loads((ROOT/'repeat_off_readonly_20261027_133354_321429/result.json').read_bytes())['snapshots'][-1]['position']));cube=m.geom_bodyid==m.body('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m)
        for offset in (offsets() if args.all_offsets else [np.zeros(16,int)]):
            worst={};maxself=maxsup=0.;first=None
            for i,q in enumerate(goals):
                d.qpos[:16]=ticks_to_sim(q+offset);d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.))
                for pair,depth in sp.items():worst[pair]=max(worst.get(pair,0.),depth)
                if first is None and (maxself>1e-6 or maxsup>.15):first=i+1
            row=dict(book_front_mm=front,offset_ticks=offset.tolist(),max_self_mm=maxself,max_support_mm=maxsup,worst_self_pairs_mm=worst,first_failure_sample=first,passed=maxself<=1e-6 and maxsup<=.15);reports.append(row)
            if first is not None:print(json.dumps(row),flush=True)
            if first is not None:break
        print('STATIC',front,sum(r['passed'] for r in reports if r['book_front_mm']==front),flush=True)
        if not reports[-1]['passed']:break
    result=dict(status='loaded_path_unloaded_geometry_diagnostic',source=source.relative_to(ROOT).as_posix(),cases=reports,all_offsets_requested=args.all_offsets,all_passed=all(r['passed'] for r in reports),hardware_execution_allowed=False,devices_opened=False,elapsed_seconds=time.perf_counter()-before)
    (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
