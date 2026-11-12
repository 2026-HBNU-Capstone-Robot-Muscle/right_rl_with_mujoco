"""Print compact summaries of offline SAC cube diagnostics."""

import json
from pathlib import Path

root = Path(__file__).resolve().parent
for name in (
    "sac_fit_contact_real_cube_100g_result.json",
    "sac_fit_contact_real_cube_100g_near_measured_sequence_placement_result.json",
    "sac_sim_measured_cube_real_cube_100g_result.json",
):
    data = json.loads((root / name).read_text(encoding="utf-8"))
    if "measured_contact_diagnostic" in data:
        print(name, data["measured_contact_diagnostic"])
    else:
        print(name, {key: data.get(key) for key in ("steps_run", "ever_success", "recorded_start_test", "measured_cube_input")})
