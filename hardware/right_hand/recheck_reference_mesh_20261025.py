"""Re-evaluate numeric proposals under corrected compiled mesh geometry.

Old static flags are deliberately ignored. No hardware approval is produced.
"""
import hashlib
import json
import mujoco
import numpy as np
from adapt_original_sequence_clearance_20261024 import build,OUT
from review_original_sequence_20261024 import load_source
from hand_motor_mapping import ticks_to_sim,hw_to_sim


def main():
    m,manifest=build();d=mujoco.MjData(m);cube=m.geom('cube').id
    m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def check(q):
        d.qpos[:16]=q;d.qpos[-7:]=[10,10,10,1,0,0,0];d.qvel[:]=0;mujoco.mj_forward(m,d)
        found=[]
        for c in d.contact:
            if c.dist>=-1e-7:continue
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if not (ha and hb or a in supports and hb or b in supports and ha):continue
            names=[m.geom(a).name,m.geom(b).name]
            kind='middle_ring_CAD_mesh' if all(n.startswith('adapt_mf_rf_') for n in names) else 'native_or_support'
            found.append(dict(pair=names,model_overlap_mm=-float(c.dist)*1000,kind=kind))
        return found
    names,hw,source=load_source();original=[]
    for i,q in enumerate(hw_to_sim(hw)):
        c=check(q);original.append(dict(stage=i,name=names[i],contacts=c))
    results=[]
    for filename in ('horizontal_reference_primitive.json','horizontal_reference_primitive_to6.json'):
        p=OUT/filename;blob=p.read_bytes();data=json.loads(blob);bad=[];max_depth=0.;max_mr=0.
        for index,q in enumerate(ticks_to_sim(data['goal_ticks'])):
            c=check(q)
            if c:
                max_depth=max(max_depth,max(v['model_overlap_mm'] for v in c))
                max_mr=max(max_mr,max([v['model_overlap_mm'] for v in c if v['kind']=='middle_ring_CAD_mesh'] or [0]))
                if len(bad)<20:bad.append(dict(index=index,contacts=c))
        row=dict(proposal=filename,proposal_sha256=hashlib.sha256(blob).hexdigest(),
            samples=len(data['goal_ticks']),clear_under_corrected_mesh=not bool(bad),
            maximum_model_overlap_mm=max_depth,maximum_middle_ring_overlap_mm=max_mr,first_failures=bad)
        results.append(row);print('CORRECTED_MESH_REFERENCE',filename,'clear',not bool(bad),'max_overlap_mm',max_depth,flush=True)
    grasps=[]
    p=OUT/'regrasp_configuration_search_v3.json'
    if p.exists():
        data=json.loads(p.read_bytes())
        for row in data['candidates']:
            if not row['geometric_grasp_candidate']:continue
            contacts=check(ticks_to_sim(row['goal_ticks']))
            grasps.append(dict(seed=row['seed'],goal_ticks=row['goal_ticks'],
                clear_under_corrected_mesh=not bool(contacts),contacts=contacts))
    report=dict(model_revision=manifest['middle_ring_mesh_model_revision'],
        source=source,original_endpoints=original,reference_paths=results,regrasp_goal_proposals=grasps,
        previous_sphere_proxy_analyses_not_reused_as_proof=True,compiled_mesh_type_asserted_by_builder=True,
        static_check_only=True,cube_dynamics_revalidation_pending=True,
        hardware_execution_allowed=False,hardware_commands_sent=0)
    (OUT/'corrected_mesh_reference_recheck.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('CORRECTED_REGRASP_GOALS',[(r['seed'],r['clear_under_corrected_mesh']) for r in grasps],flush=True)


if __name__=='__main__':main()
