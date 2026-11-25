"""Compare real encoder poses to commanded poses in original completed runs."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/original_measured_replay_v1'
def main():
    OUT.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'scene.xml');d=mujoco.MjData(m);palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    def collision(q):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,10];d.qvel[:]=0;mujoco.mj_forward(m,d);maximum=0.;pairs=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                depth=max(0.,-float(c.dist)*1000);maximum=max(maximum,depth)
                if depth>.1:pairs.append(dict(pair=[m.geom(a).name,m.geom(b).name],depth_mm=depth))
        return dict(max_self_overlap_mm=maximum,pairs=pairs)
    results=[]
    for name in ('leap_sequence_flip_20261011_051350_713486.json','leap_sequence_flip_20261011_051518_433595.json','leap_anchor_capture_result.json'):
        path=ROOT/name;r=json.loads(path.read_bytes());samples=r['samples'];rows=[]
        for i,s in enumerate(samples):
            if 'actual' not in s:raise RuntimeError('Need original raw actual motor positions')
            rows.append(dict(index=i,phase=s['phase'],t=s.get('t'),actual_ticks=s['actual'],goal_ticks=s['goal'],actual=collision(s['actual']),commanded=collision(s['goal'])))
        phases=[]
        for phase in dict.fromkeys(s['phase'] for s in samples):
            group=[v for v in rows if v['phase']==phase];final=group[-1];phases.append(dict(phase=phase,samples=len(group),maximum_actual_self_overlap_mm=max(v['actual']['max_self_overlap_mm'] for v in group),maximum_goal_self_overlap_mm=max(v['commanded']['max_self_overlap_mm'] for v in group),last_actual_ticks=final['actual_ticks'],last_goal_ticks=final['goal_ticks'],last_actual_collision=final['actual'],last_goal_collision=final['commanded']))
        record=dict(source=name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),status=r['status'],phases=phases,samples=rows,real_devices_opened=False,hardware_execution_allowed=False);results.append(record);(OUT/(path.stem+'_collision_review.json')).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8');print(json.dumps(dict(source=name,phases=phases)),flush=True)
    report=dict(status='original_actual_vs_commanded_pose_collision_review',source_files=[r['source'] for r in results],comparison_uses_current_corrected_motor_order=True,model_collisions_are_not_independent_proof_of_real_contact=True,real_devices_opened=False,hardware_execution_allowed=False);(OUT/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
