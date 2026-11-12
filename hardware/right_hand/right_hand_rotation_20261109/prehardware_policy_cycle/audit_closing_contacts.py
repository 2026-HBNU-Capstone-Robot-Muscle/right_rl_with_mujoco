"""Check all anatomical contact bodies, without changing task acceptance."""
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from handoff_support_wrench import contact_wrench


def main():
    m,_,_,_,provenance=assets('active');g=Guard(m);rows=[]
    for name in ('handoff_integer_endpoint_close_v4','handoff_integer_endpoint_close_v5','handoff_integer_load_first_v6'):
        path=OUT/'next_cycle'/name/'end_state.npz';d=restore(m,dict(np.load(path,allow_pickle=False)));w=contact_wrench(m,d,g.hand,True)
        contacts=[];families={}
        for geom,detail in w['by_geom'].items():
            gid=m.geom(geom).id;body=m.body(int(m.geom_bodyid[gid])).name
            family=next((s for s in ('if','mf','rf','th') if body.startswith(s+'_')),'other')
            families[family]=families.get(family,0.)+detail['normal_N']
            contacts.append(dict(geom=geom,body=body,finger=family,normal_N=detail['normal_N'],world_force_N=detail['force_world_N'].tolist()))
        rows.append(dict(source=str(path.relative_to(OUT)),sha256=sha(path),contacts=contacts,
                         anatomical_normal_force_sums_N=families,constraint_force_error=w['constraint_force_error']))
    result=dict(status='anatomical_contact_identity_audit',states=rows,model_provenance=provenance,
                task_acceptance_criteria_changed=False,devices_opened=0,hardware_execution_allowed=False)
    save_json(OUT/'next_cycle/handoff_refinement_20261110/closing_contact_audit.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
