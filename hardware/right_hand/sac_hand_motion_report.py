"""Measure ID40/camera relative motion within the combined sequence capture."""

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
folder = ROOT / "sequence_vision_20261006_043709_528624"
data = json.loads((folder / "result.json").read_text(encoding="utf-8"))
placed_at = data["placement_image_captured_at_unix_s"]
rows = [f for f in data["frames"] if f.get("pose") and "t_hand" in f["pose"]]
phases = sorted({f["phase"] for f in rows})
result = []
for phase in phases:
    group = [f for f in rows if f["phase"] == phase]
    positions = np.asarray([f["pose"]["t_hand"] for f in group])
    median = np.median(positions, axis=0)
    result.append({
        "phase": phase,
        "count": len(group),
        "time_from_placement_s": float(np.median([f["captured_at_unix_s"] - placed_at for f in group])),
        "median_t_hand_camera_m": median.tolist(),
        "spread_from_median_mm": float(np.percentile(np.linalg.norm(positions - median, axis=1), 90) * 1000),
    })
if result:
    base = np.asarray(next(r["median_t_hand_camera_m"] for r in result if r["phase"] == "preflight"))
    for r in result:
        r["shift_from_preflight_mm"] = float(np.linalg.norm(np.asarray(r["median_t_hand_camera_m"]) - base) * 1000)
(ROOT / "sac_hand_motion_report_result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, indent=2))
