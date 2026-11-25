"""Summarize cube-contact bodies from the archived offline sequence replay."""

import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "sac_sequence_replay_sim_result.json"


def main():
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    candidates = data["candidate_results"]
    stages = {}
    for stage in range(7):
        rows = [r for candidate in candidates for r in candidate["stage_endpoints"]
                if r["stage"] == stage]
        touching = Counter(body for row in rows for body in row["touching_bodies"])
        stages[f"stage_{stage}"] = {
            "candidates": len(rows),
            "any_finger_contact": sum(any(body not in ("palm", "world")
                                           for body in row["touching_bodies"]) for row in rows),
            "two_distinct_fingers_contact": sum(
                len({body.split("_")[0] for body in row["touching_bodies"]
                     if body.split("_")[0] in ("th", "if", "mf", "rf")}) >= 2
                for row in rows),
            "touching_body_count": dict(touching),
            "all_command_steps": sum(c["contact_steps"].get(f"stage_{stage}", {}).get("steps", 0)
                                     for c in candidates),
            "any_finger_command_steps": sum(
                c["contact_steps"].get(f"stage_{stage}", {}).get("any_finger_steps", 0)
                for c in candidates),
            "two_finger_command_steps": sum(
                c["contact_steps"].get(f"stage_{stage}", {}).get("two_finger_steps", 0)
                for c in candidates),
        }
    report = {
        "status": "offline_cube_contact_audit",
        "motor_commands_sent": 0,
        "source": SOURCE.name,
        "stages": stages,
        "caveat": "Approximate MuJoCo collision model. Counts include all commanded interpolation steps but cannot establish real physical contact.",
    }
    (ROOT / "sac_sequence_contact_audit_result.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
