"""Fresh actual-OFF recovery screen for the diagnostic; no physical devices."""
import argparse
import json
import os
import sys
from pathlib import Path
os.environ["JAX_PLATFORMS"] = "cpu"
from work_grip_stage2_plan_20261112 import ROOT, WorkGripStage2Plan, hashes
from work_grip_stage2_gate_20261112 import make_review


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("request")
    parser.add_argument("output")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.relative_to(ROOT)
    if output.exists():
        raise RuntimeError("Refuse to overwrite proof")
    plan = WorkGripStage2Plan(args.request)
    before = hashes()
    import screen_calibrated_empty_margin_20261028 as screen
    screen.OfflineRecovery = WorkGripStage2Plan
    old = sys.argv
    folder = output.parent / "empty_recovery_screen"
    try:
        sys.argv = ["empty", "--request", str(plan.SOURCE), "--output-folder", folder.relative_to(ROOT).as_posix()]
        screen.main()
    finally:
        sys.argv = old
    report = make_review(plan, json.loads((folder / "review.json").read_bytes()), before)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("WORK GRIP STAGE2 REVIEW COMPLETE", flush=True)


if __name__ == "__main__":
    main()
