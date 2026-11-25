"""Inspect actual checkpoint contact ownership, forces and available pinch sites."""
import json
import numpy as np
from native_cycle import OUT,assets,restore,Guard,save_json,sha
from whole_hand_support import sample_model


def main():
    base=OUT/'next_cycle/handoff_refinement_20261110';model,_,_,_,_=assets('active_ccd50');cube=model.geom('cube').id;rows=[]
    for name in ('first90_finish_checkpoint','next90_recenter_checkpoint'):
        path=base/name/'state.npz';data=restore(model,dict(np.load(path,allow_pickle=False)));guard=Guard(model)
        row=dict(source=path.relative_to(OUT).as_posix(),source_sha256=sha(path),support=sample_model(model,data,guard.hand,cube),
            fingertip_centers_cube_frame_mm={finger:((data.geom_xpos[model.geom(finger+'_tip').id]-data.geom_xpos[cube])@data.geom_xmat[cube].reshape(3,3)*1000).tolist() for finger in ('if','mf','rf','th')},
            contacts=[dict(geom=model.geom(int(c.geom2) if int(c.geom1)==cube else int(c.geom1)).name,world_position=c.pos.tolist(),distance_mm=float(c.dist*1000))
                for c in data.contact if cube in (int(c.geom1),int(c.geom2))])
        rows.append(row)
    save_json(base/'bridge_checkpoint_contacts.json',dict(status='actual_checkpoint_contact_inspection',rows=rows,devices_opened=0))
    print(json.dumps([dict(source=row['source'],normal_N={key:value['normal_N'] for key,value in row['support']['by_geom'].items()},tip_centers=row['fingertip_centers_cube_frame_mm']) for row in rows]),flush=True)


if __name__=='__main__':main()
