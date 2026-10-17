#!/usr/bin/env python3
"""Convert Koide's camera->LiDAR pose to FAST-LIVO2 LiDAR->camera extrinsics."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calib_json", type=Path, help="Koide calibration output calib.json")
    parser.add_argument("output_yaml", type=Path, help="New FAST-LIVO2 visual config")
    args = parser.parse_args()

    if args.output_yaml.exists():
        parser.error(f"Output exists; choose a new file instead: {args.output_yaml}")

    result = json.loads(args.calib_json.read_text(encoding="utf-8"))
    pose = result["results"]["T_lidar_camera"]
    if len(pose) != 7 or not all(math.isfinite(float(value)) for value in pose):
        parser.error("T_lidar_camera must contain seven finite values")
    translation_lidar_camera = np.asarray(pose[:3], dtype=float)
    quat_xyzw = np.asarray(pose[3:], dtype=float)
    if not 0.99 <= np.linalg.norm(quat_xyzw) <= 1.01:
        parser.error("Calibration quaternion is not close to unit length")

    # Koide: p_lidar = R_lc @ p_camera + t_lc.
    # FAST-LIVO2: p_camera = Rcl @ p_lidar + Pcl.
    rotation_camera_lidar = Rotation.from_quat(quat_xyzw).as_matrix().T
    translation_camera_lidar = -rotation_camera_lidar @ translation_lidar_camera

    base_dir = Path(__file__).resolve().parents[1] / "src" / "FAST-LIVO2" / "config"
    camera = yaml.safe_load((base_dir / "camera_d435i_261722072542.yaml").read_text(encoding="utf-8"))["/**"]["ros__parameters"]["camera"]
    measured = result["camera"]["intrinsics"]
    expected = [camera["fx"], camera["fy"], camera["cx"], camera["cy"]]
    if len(measured) < 4 or max(abs(float(a) - b) for a, b in zip(measured[:4], expected)) > 1.0:
        parser.error("Calibration intrinsics differ from the saved D435i 640x480 profile; check camera profile and camera config")
    distortion = result["camera"].get("distortion_coeffs", [])
    if any(abs(float(value)) > 1e-3 for value in distortion):
        parser.error("Calibration reports nonzero distortion; update and validate FAST-LIVO2 camera config before using it")

    params = yaml.safe_load((base_dir / "mid360s_d435i_lio.yaml").read_text(encoding="utf-8"))
    mapping = params["/**"]["ros__parameters"]
    mapping["common"]["img_en"] = 1
    mapping["extrin_calib"]["Rcl"] = [float(value) for value in rotation_camera_lidar.reshape(-1)]
    mapping["extrin_calib"]["Pcl"] = [float(value) for value in translation_camera_lidar]
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(yaml.safe_dump(params, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"Wrote {args.output_yaml}")
    print("Rcl (LiDAR to camera optical):", mapping["extrin_calib"]["Rcl"])
    print("Pcl (LiDAR to camera optical, m):", mapping["extrin_calib"]["Pcl"])
    print("Check projection in the calibration viewer before using this mapping config.")


if __name__ == "__main__":
    main()
