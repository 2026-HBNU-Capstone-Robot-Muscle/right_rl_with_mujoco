"""Recheck archived Oct 7 same-session marker bridges without hardware."""

import json
from pathlib import Path

from sac_registration_from_anchor import estimate


ROOT = Path(__file__).resolve().parent


def main():
    rows = []
    for folder in sorted(ROOT.glob("sequence_vision_20261007_*")):
        if not (folder / "placement_at_space.png").exists():
            continue
        for name in ("stage_0_not_ready.png", "preflight_not_ready.png"):
            if not (folder / name).exists():
                continue
            try:
                report = estimate(folder, empty_image=name)
                rows.append({"folder": folder.name, "empty": name,
                             "status": report["status"],
                             "cube_center_in_hand_marker_m": report["cube_center_in_hand_marker_m"],
                             "cube_marker_count": report["cube_marker_count"],
                             "cube_reprojection_px": report["cube_reprojection_px"]})
            except Exception as exc:
                rows.append({"folder": folder.name, "empty": name, "error": str(exc)})
    result = {"rows": rows, "hardware_authorized": False}
    (ROOT / "sac_oct7_pair_scan_result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
