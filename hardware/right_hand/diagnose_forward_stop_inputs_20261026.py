"""Saved camera-frame and OFF-contact diagnostics only."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/loaded_finish_review_0057_20261026'
FOLDERS=('forward_cube_placement_20261026_031258_470086','forward_cube_placement_20261026_031425_026161','forward_cube_placement_20261026_031746_921869','forward_cube_placement_20261026_031956_975491')
def cameras():
    rows=[]
    for name in FOLDERS:
        p=ROOT/name;r=json.loads((p/'result.json').read_bytes());req=p/'fresh_off_request.json'
        row=dict(folder=name,error=r.get('error'),camera_image_signal=r.get('camera_image_signal'),camera_off_adjustment_waits=r.get('camera_off_adjustment_waits'),position=json.loads(req.read_bytes())['off_state']['position'] if req.exists() else None)
        f=p/'camera/frames.json'
        if f.exists():
            frames=json.loads(f.read_bytes())['frames'];indices=[i for i,v in enumerate(frames) if not v.get('image_signal_usable')]
            near=sorted(set(j for i in indices for j in range(max(0,i-2),min(len(frames),i+3))))
            row['bad_frame_neighborhoods']=[dict(index=i,**frames[i]) for i in near]
        rows.append(row)
    (OUT/'camera_stop_input_review.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(rows,ensure_ascii=False,indent=2))
def contacts():
    import numpy as np,mujoco
    from mujoco_playground import registry
    import screen_loaded_quarter_finish_20261025 as c
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    p=ROOT/FOLDERS[-1];plan=c.BookGraspPlan(p/'fresh_off_request.json');raw=plan.source_position()
    m,manifest=c.build_cad(env,OUT/'interrupted_off_contact_diagnostic.xml',book_projection_mm=-10.)
    c.correct_model(m,raw);m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    inspect=c.cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
    d=mujoco.MjData(m)
    def pose(q):
        d.qpos[:16]=c.ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
        support,self_pairs=inspect(d)
        return dict(support=support,self_contacts=c.classify_photo_initial(c.classify_self_contacts(self_pairs),initial_release=True))
    q=plan.plan(raw);samples=[]
    for idx,(phase,v) in enumerate([('start',raw)]+q[:100]):
        if idx%8==0:samples.append(dict(index=idx,phase=phase,ticks=v.tolist(),**pose(v)))
    row=dict(raw_ticks=raw.tolist(),bounds={c.CASING_PAIR:c.CASING_INITIAL_LIMIT_MM,c.EXTRA_RING_PAIR:c.EXTRA_RING_INITIAL_LIMIT_MM,c.THUMB_NEIGHBOR_PAIR:c.THUMB_NEIGHBOR_LIMIT_MM},nominal=pose(raw),release_samples=samples,hardware_commands_sent=0)
    (OUT/'interrupted_off_contact_diagnostic.json').write_text(json.dumps(row,indent=2),encoding='utf-8');print(json.dumps(row,indent=2))
if __name__=='__main__':contacts() if '--contacts' in sys.argv else cameras()
