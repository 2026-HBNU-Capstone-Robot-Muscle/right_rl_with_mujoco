"""Compare candidate joint-command lineage with the actually run stage-1 path."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
WORK = ROOT / "right_hand_rotation_20261109/prehardware_policy_cycle"
BASE = WORK / "next_cycle/handoff_refinement_20261110"
OUT = ROOT / "right_hand_rotation_20261111/stage2_preparation"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    old = dict(np.load(ROOT / "right_hand_rotation_20261104/forward_hold/candidate.npz", allow_pickle=False))
    end = old["goals"][-1]
    rows = []
    paths = [ROOT / "right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/candidate.npz",
             BASE / "initial_second_turn_hold_replay/research_goals.npz",
             BASE / "initial_to_reconfigured_first90/research_goals.npz",
             BASE / "initial_to_reached_workstation90/research_goals.npz",
             BASE / "robust_mu025_complete_cycle75ms_replay_v39/research_goals.npz"]
    for path in paths:
        values = dict(np.load(path, allow_pickle=False))
        goals = values["goals"]
        n = min(len(old["goals"]), len(goals))
        diffs = np.max(abs(goals[:n] - old["goals"][:n]), axis=1)
        nearest = np.max(abs(goals - end), axis=1)
        i = int(np.argmin(nearest))
        row = {"path": path.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
               "keys": list(values), "goals": len(goals), "start_ticks": values.get("start", []).tolist(),
               "identical_prefix_length": int(np.flatnonzero(diffs)[0]) if np.any(diffs) else n,
               "nearest_to_actual_stage1_goal_index": i, "nearest_maximum_tick_difference": float(nearest[i]),
               "nearest_goal": goals[i].tolist()}
        if len(goals) == 5089:
            protected = np.zeros(len(goals), dtype=bool)
            protected[:1417] = True
            protected[3351:3405] = True
            protected[-54:] = True
            for keep in (1, 3, 6, 10):
                count = run = 0
                previous = None
                for j, q in enumerate(goals):
                    run = run + 1 if previous is not None and np.array_equal(q, previous) else 1
                    previous = q
                    count += bool(protected[j] or run <= keep)
                row[f"deduplicated_count_max_run_{keep}"] = count
        rows.append(row)
    result = {"status": "candidate_command_lineage_audit", "stage1_goals": len(old["goals"]),
              "stage1_final_goal": end.tolist(), "rows": rows, "devices_opened": 0}
    (OUT / "source_audit.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result))
    for path in (ROOT / "right_hand_rotation_20261105/canonical_body_grip/continuous_handoff/review.json",
                 BASE / "initial_second_turn_hold_replay/review.json",
                 BASE / "initial_to_reconfigured_first90/review.json"):
        report = json.loads(path.read_bytes())
        print(json.dumps({"report": path.relative_to(ROOT).as_posix(),
            "scalars": {k: v for k, v in report.items() if not isinstance(v, (list, dict))},
            "segments": report.get("segments"),
            "structures": {k: (list(v) if isinstance(v, dict) else len(v)) for k, v in report.items() if isinstance(v, (list, dict))}}, ensure_ascii=False))


if __name__ == "__main__":
    main()
