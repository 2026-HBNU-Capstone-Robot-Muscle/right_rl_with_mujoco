"""Local FK sensitivity at saved failures. Candidates require free-cube replay."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json
from pathlib import Path
import mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from hand_motor_mapping import SIM_TO_MOTOR_IDS,TICK,sim_to_hw
from sac_cube_model import set_cube_physics
from screen_loaded_repeat_20261026 import cached_contacts
from slow_thumb_seam_resolution_20261026 import classify
from calibrated_quarter_plan_20261028 import LOADED_PATH
from simulate_handoff_yaw_continuation_20261028 import OUT

def main():
    out=OUT/'contact_clearance_candidates_v2_precise';out.mkdir(exist_ok=True)
    source=OUT/'failure_contacts_precise4';report=json.loads((source/'review.json').read_bytes())
    failed=[r for r in report['cases'] if r['stop']=='cube_overlap'];assert failed
    states=[np.load(source/r['trace'])['failure_contact_qpos'] for r in failed]
    z=np.load(LOADED_PATH);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    m,_=build(registry.load('LeapCubeFaceUp',config=cfg),out/'probe_scene.xml',book_projection_mm=-10.)
    correct_model(m,z['start']);set_cube_physics(m,.055,.1);d=mujoco.MjData(m)
    cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith(('support_','leap_mount_collision')) or m.geom(g).name=='floor'}
    inspect=cached_contacts(m,hand,support);lo,hi=limits();rank=[];original_contacts=[]
    for state in states:
        d.qpos[:]=state;mujoco.mj_forward(m,d);pairs=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if cube in (a,b) and c.dist<0:pairs.append(dict(other=m.geom(b if a==cube else a).name,depth_mm=-float(c.dist)*1000,
                position_in_cube_mm=(d.geom_xmat[cube].reshape(3,3).T@(c.pos-d.geom_xpos[cube])*1000).tolist()))
        original_contacts.append(sorted(pairs,key=lambda v:-v['depth_mm'])[:5])
    for motor in range(1,17):
        j=SIM_TO_MOTOR_IDS.index(motor)
        for delta in (-12,-8,-4,4,8,12):
            depths=[];maximum_self=maximum_support=0.;native_ok=True
            for state in states:
                d.qpos[:]=state;d.qpos[j]+=delta*TICK;mujoco.mj_forward(m,d)
                a,b=inspect(d);maximum_self=max(maximum_self,max(classify(b).values(),default=0.));maximum_support=max(maximum_support,max(a.values(),default=0.))
                native=sim_to_hw(d.qpos[:16])/TICK;native_ok &=bool(np.all(native>=lo) and np.all(native<=hi))
                depth=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
                depths.append(depth)
            rank.append(dict(motor=motor,delta_ticks=delta,delta_degrees=delta*360/4096,
                maximum_cube_depth_mm=max(depths),depths_mm=depths,max_self_mm=maximum_self,max_support_mm=maximum_support,native_ok=native_ok))
    rank.sort(key=lambda r:(not(r['native_ok'] and r['max_self_mm']<=1e-6 and r['max_support_mm']<=.15),r['maximum_cube_depth_mm'],abs(r['delta_ticks'])))
    # A middle-finger and a thumb contact can fail in different snapshots.
    # Rank small two-joint combinations as well as single-joint changes.
    pair_rank=[]
    for middle in (9,10,11,12):
        for thumb in (1,2,3,4):
            for md in (-8,-4,4,8):
                for td in (-8,-4,4,8):
                    depths=[];maximum_self=maximum_support=0.;native_ok=True
                    for state in states:
                        d.qpos[:]=state
                        d.qpos[SIM_TO_MOTOR_IDS.index(middle)]+=md*TICK
                        d.qpos[SIM_TO_MOTOR_IDS.index(thumb)]+=td*TICK
                        mujoco.mj_forward(m,d)
                        a,b=inspect(d);maximum_self=max(maximum_self,max(classify(b).values(),default=0.));maximum_support=max(maximum_support,max(a.values(),default=0.))
                        native=sim_to_hw(d.qpos[:16])/TICK;native_ok &=bool(np.all(native>=lo) and np.all(native<=hi))
                        depths.append(max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.]))
                    pair_rank.append(dict(offsets={str(middle):md,str(thumb):td},maximum_cube_depth_mm=max(depths),
                        depths_mm=depths,max_self_mm=maximum_self,max_support_mm=maximum_support,native_ok=native_ok))
    choices=[dict(v,offsets={str(v['motor']):v['delta_ticks']}) for v in rank]+pair_rank
    choices.sort(key=lambda r:(not(r['native_ok'] and r['max_self_mm']<=1e-6 and r['max_support_mm']<=.15),
        r['maximum_cube_depth_mm']>=.9,sum(abs(v) for v in r['offsets'].values()),r['maximum_cube_depth_mm']))
    candidates=[]
    for value in choices:
        if len(candidates)>=4:break
        if value['maximum_cube_depth_mm']>=1 or not value['native_ok'] or value['max_self_mm']>1e-6 or value['max_support_mm']>.15:continue
        offsets=value['offsets'];rows=[];previous=z['start'].copy();labels=[]
        counts={p:int(np.count_nonzero(z['phases']==p)) for p in np.unique(z['phases'])};seen={}
        valid=True
        for q,phase in zip(z['goals'],z['phases']):
            phase=str(phase);seen[phase]=seen.get(phase,0)+1
            if phase=='grasp':fraction=seen[phase]/counts[phase]
            elif phase in ('grasp_hold','yaw','yaw_hold'):fraction=1.
            elif phase=='release':fraction=1.-seen[phase]/counts[phase]
            else:fraction=0.
            target=q.copy()
            for motor,delta in offsets.items():target[int(motor)-1]+=int(np.rint(delta*fraction))
            if np.any(target<lo) or np.any(target>hi):valid=False;break
            n=max(1,int(np.ceil(max(abs(target-previous))/2)))
            rows.extend(np.rint([previous+(target-previous)*i/n for i in range(1,n+1)]).astype(int));labels.extend([phase]*n);previous=target
        if not valid:continue
        rows=np.asarray(rows);assert np.max(abs(np.diff(np.vstack([z['start'],rows]),axis=0)))<=2
        name='_'.join(f'ID{motor}_{delta:+d}' for motor,delta in offsets.items())+'.npz';np.savez_compressed(out/name,start=z['start'],goals=rows,phases=np.asarray(labels),sample_period_s=.075)
        candidates.append(dict(**value,path=(out/name).relative_to(Path(__file__).resolve().parent).as_posix(),goals=len(rows)))
    result=dict(status='offline_local_contact_clearance_FK_hypotheses',failed_sources=[r['seed'] for r in failed],
        source_report_sha256=hashlib.sha256((source/'review.json').read_bytes()).hexdigest(),
        original_contacts=original_contacts,ranking=rank,pair_ranking=pair_rank,candidates=candidates,
        uses_pre_step_state_matching_collision_evaluation=True,
        fixed_cube_pose_used_only_for_local_FK_sensitivity=True,dynamic_validation_required=True,
        old_model_contact_thresholds_unchanged=True,original_reference_unchanged=True,hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(contacts=original_contacts,top=rank[:6],candidates=candidates)),flush=True)

if __name__=='__main__':main()
