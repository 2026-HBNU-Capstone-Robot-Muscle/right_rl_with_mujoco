"""Compare the installed MuJoCo palm axes with the published LEAP V1 URDF.

Read-only: opens no camera or motor port. The URDF coordinates below are from
https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf
The published joint numbers are zero-based CAD labels, not Dynamixel IDs.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parent
XML = ROOT.parent / "work/Leap_Test/train/env/xmls/leap_rh_mjx.xml"
OFFICIAL = {
    "index": {"urdf_joint": 1, "motor_id": 5, "sim_body": "if_bs",
              "palm_xyz_m": [-0.00709524965, 0.02305775613, -0.01872240358]},
    "middle": {"urdf_joint": 5, "motor_id": 9, "sim_body": "mf_bs",
               "palm_xyz_m": [-0.00709524965, -0.022392243865, -0.01872240358]},
    "ring": {"urdf_joint": 9, "motor_id": 13, "sim_body": "rf_bs",
             "palm_xyz_m": [-0.00709524544, -0.067842243865, -0.01872240767]},
    "thumb": {"urdf_joint": 12, "motor_id": 1, "sim_body": "th_mp",
              "palm_xyz_m": [-0.06939524965, -0.001242243865, -0.02162240358]},
}


def audit():
    xml = ET.parse(XML)
    palm = next(b for b in xml.iter("body") if b.get("name") == "palm")
    bodies = {b.get("name"): b for b in palm if b.tag == "body"}
    rows = {}
    for finger, item in OFFICIAL.items():
        source = np.asarray(item["palm_xyz_m"], dtype=float)
        installed = np.fromstring(bodies[item["sim_body"]].get("pos"), sep=" ")
        rows[finger] = {
            **item,
            "installed_palm_xyz_m": installed.tolist(),
            "rounding_difference_mm": float(np.linalg.norm(installed - source) * 1000),
        }
    source_span = (np.asarray(OFFICIAL["index"]["palm_xyz_m"])
                   - np.asarray(OFFICIAL["middle"]["palm_xyz_m"]))
    maximum_rounding_mm = max(x["rounding_difference_mm"] for x in rows.values())
    row = {
        "status": "official_finger_axes_match_installed_model" if maximum_rounding_mm < 0.2 else "axis_mismatch",
        "source": "https://github.com/leap-hand/LEAP_Hand_Sim/blob/master/assets/leap_hand/robot.urdf",
        "installed_xml": str(XML),
        "fingers": rows,
        "official_index_to_middle_axis_spacing_mm": float(np.linalg.norm(source_span) * 1000),
        "maximum_axis_rounding_difference_mm": maximum_rounding_mm,
        "cad_joint9_is": "ring MCP; real Dynamixel ID9 is middle MCP (CAD joint5)",
        "custom_id40_id41_mount_in_official_cad": False,
        "custom_support_in_official_cad": False,
        "registration_conclusion": "Official geometry fixes the motor-to-motor axes but cannot alone determine the user's marker-to-axis transform or custom support contact.",
        "hardware_authorized": False,
    }
    assert maximum_rounding_mm < 0.2
    return row


if __name__ == "__main__":
    result = audit()
    path = ROOT / "sac_official_geometry_audit_result.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
