"""Measured root trial and proposed thumb preparation: offline kinematics only."""
import os
os.environ.setdefault("MUJOCO_GL", "egl")
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from hand_motor_mapping import ticks_to_sim, SIM_TO_MOTOR_IDS
from offline_housing_proxy_model import build_with_middle_thumb_hulls

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "right_hand_root_success_20261020"

def main():
    source = OUT / "source_result.json"
    r = json.loads(source.read_bytes())
    assert r["status"] == "empty_root_lift_complete"
    assert r["joint_mapping"]["motor_ids_in_sim_order"] == list(SIM_TO_MOTOR_IDS)
    targetfile = ROOT / "full_mapping_pose_review_20261019/inward_quantized/housing_sensitivity/path_OFFLINE_ONLY.json"
    target = np.array(json.loads(targetfile.read_text())["start_ticks"])
    cfg = registry.get_default_config("LeapCubeFaceUp")
    cfg.impl = "jax"
    env = registry.load("LeapCubeFaceUp", config=cfg)
    m, manifest = build_with_middle_thumb_hulls(env)
    d = mujoco.MjData(m)
    hand = {m.body("palm").id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:
            hand.add(b)
    supports = {m.geom(n).id for n in ("support_glass","support_wood")}
    bounds = m.jnt_range[m.actuator_trnid[:,0]]

    def inspect(raw):
        q = ticks_to_sim(raw)
        d.qpos[:16] = q
        d.qpos[-7:-4] = [0,0,2]
        d.qpos[-4:] = [1,0,0,0]
        mujoco.mj_forward(m,d)
        contacts = []
        for c in d.contact:
            a,b = int(c.geom1),int(c.geom2)
            if c.dist >= -1e-5:
                continue
            ha,hb = int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if (ha and hb) or (a in supports and hb) or (b in supports and ha):
                contacts.append(dict(a=m.geom(a).name,b=m.geom(b).name,
                    overlap_mm=float(-c.dist*1000),support=a in supports or b in supports))
        return dict(contacts=contacts, outside_native_range_ids=[
            int(SIM_TO_MOTOR_IDS[i]) for i in range(16)
            if not bounds[i,0] <= q[i] <= bounds[i,1]])

    def summarize(rows):
        return dict(samples=len(rows),
            self_overlap_samples=sum(any(not c["support"] for c in x["contacts"]) for x in rows),
            support_overlap_samples=sum(any(c["support"] for c in x["contacts"]) for x in rows),
            max_self_overlap_mm=max([c["overlap_mm"] for x in rows for c in x["contacts"] if not c["support"]] or [0]),
            max_support_overlap_mm=max([c["overlap_mm"] for x in rows for c in x["contacts"] if c["support"]] or [0]),
            outside_native_range_samples=sum(bool(x["outside_native_range_ids"]) for x in rows))

    measured = [dict(sample=i,**inspect(s["actual"])) for i,s in enumerate(r["samples"])]
    start = np.array(r["final_on_state"]["position"])
    # Investigate ordering without changing limits or clipping the measured
    # thumb. All candidates remain non-executable, including zero-overlap ones.
    candidates = {}
    orders = [("simultaneous",[(0,1,2,3)])]
    orders += [("ids_"+"_".join(str(i+1) for i in order),[(i,) for i in order])
               for order in itertools.permutations(range(4))]
    for name,groups in orders:
        last = start.copy()
        rows = [dict(segment="start",**inspect(last))]
        for group in groups:
            end = last.copy()
            end[list(group)] = target[list(group)]
            n = max(1,int(np.ceil(np.max(abs(end-last))/5)))
            for k in range(1,n+1):
                raw = np.rint(last+(end-last)*k/n).astype(int)
                rows.append(dict(segment=[i+1 for i in group],step=k,**inspect(raw)))
            last = end
        candidates[name] = dict(**summarize(rows),
            initial_raw_ticks=start.tolist(),final_raw_ticks=last.tolist(),
            first_overlap=next((x for x in rows if x["contacts"]),None))
    alternatives = {}
    # Small thumb-tip target alternatives, kept entirely offline. Each is
    # screened over the full transition, not only its endpoint.
    for delta in (-64,-32,-16,16,32,64):
        end = start.copy()
        end[:4] = target[:4]
        end[0] += delta
        n = max(1,int(np.ceil(np.max(abs(end-start))/5)))
        rows = [inspect(np.rint(start+(end-start)*k/n).astype(int)) for k in range(n+1)]
        alternatives[str(delta)] = dict(**summarize(rows),final_raw_ticks=end.tolist(),
            final_pose=rows[-1], hardware_execution_allowed=False)
    report = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        target_sha256=hashlib.sha256(targetfile.read_bytes()).hexdigest(),
        model_manifest=manifest,
        measured_root_sweep=dict(**summarize(measured),rows=measured),
        final_on_pose=inspect(start),thumb_candidates=candidates,
        thumb_tip_target_alternatives=alternatives,
        hardware_execution_allowed=False,motor_commands_sent=0,
        limitations=["Endpoint was measured while ON; present OFF pose is unknown.",
            "Approximate geometry and static samples cannot certify physical clearance.",
            "Native thumb range violation retained; no wrapping or limits changed.",
            "Thumb candidates are not dynamic validation or executable commands."])
    (OUT/"next_preparation_screen.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(dict(measured_root_sweep=summarize(measured), final_on_pose=report["final_on_pose"],
        candidate_count=len(candidates),
        candidates_without_overlap=[name for name,v in candidates.items()
                                    if not v["self_overlap_samples"] and not v["support_overlap_samples"]],
        alternatives={k:{name:v[name] for name in ("samples","max_self_overlap_mm","max_support_overlap_mm")}
                      for k,v in alternatives.items()}),indent=2))

if __name__ == "__main__":
    main()
