#!/usr/bin/env python3
"""Check integrity and consistency of saved lab evidence; do not run ROS or Isaac Sim."""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "lab"
STREAMS = {"clock", "scan", "image", "camera_info", "odom", "tf"}
EDGES = {("odom", "base_footprint")} | {
    ("base_footprint", link)
    for link in (
        "base_link", "base_scan", "caster_back_link",
        "wheel_left_link", "wheel_right_link", "Camera_1",
    )
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def read_log(name: str) -> dict:
    return json.loads((LAB / "EK_Logs" / name).read_text(encoding="utf-8"))


def main() -> None:
    for name in ("EK_live_validation.json", "EK_fresh_process_validation.json"):
        data = read_log(name)
        require(data["missing"] == [], f"Missing streams in {name}")
        require(all(data["counts"].get(stream, 0) > 0 for stream in STREAMS),
                f"A required stream has no received messages in {name}")
        require({tuple(edge) for edge in data["tf_edges"]} == EDGES,
                f"Unexpected TF tree in {name}")
        require(data["forbidden_world_odom"] is False, f"Forbidden TF reported in {name}")
        image = data["image"]
        require((image["width"], image["height"], image["encoding"], image["frame"])
                == (640, 480, "rgb8", "Camera_1"), f"Unexpected camera sample in {name}")
        require(image["bytes"] == 640 * 480 * 3, f"Unexpected RGB byte count in {name}")
        scan = data["scan"]
        require(scan["frame"] == "base_scan" and scan["ranges"] == 3600,
                f"Unexpected lidar sample in {name}")
        require(scan["finite_returns"] == 3600, f"Unexpected finite range count in {name}")
        require(data["odom"] == {"parent": "odom", "child": "base_footprint"},
                f"Unexpected odometry frames in {name}")
        print(f"PASS: {name}: six received streams, seven expected TF edges, sensor fields")

    motion = read_log("EK_acceptance_summary.json")
    for field in ("received_all_required_streams", "tf_connected", "no_world_to_odom", "motion_pass"):
        require(motion[field] is True, f"Recorded acceptance flag failed: {field}")
    require(round(motion["forward_distance_m"], 3) == 0.361, "README travel differs from evidence")
    require(round(motion["yaw_change_rad"], 3) == 1.059, "README yaw differs from evidence")
    comparisons = read_log("EK_geometry_equivalence.json")
    require(len(comparisons) == 6, "Expected equivalence records for six checkpoints")
    for name, record in comparisons.items():
        require(record["identical"] is True and record["before"] == record["after"],
                f"Recorded geometry/physics equivalence failed: {name}")
    print("PASS: retained motion summary and six checkpoint equivalence records")
    print("Saved evidence checks passed. No new simulator execution was performed.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
