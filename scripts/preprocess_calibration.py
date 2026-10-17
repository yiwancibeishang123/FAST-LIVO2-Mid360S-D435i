#!/usr/bin/env python3
"""Preprocess five calibration bags in a fresh directory; archive old results.

CustomMsg inputs are converted offline for static calibration. This script
does not estimate extrinsics or launch any sensor drivers.
"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml


def fresh_path(root, name):
    for index in range(1, 10000):
        path = root / f"{name}_{index:02d}"
        if not path.exists():
            return path
    raise RuntimeError("No unused output name available")


def snapshot(bags):
    return {str(p): {"bytes": p.stat().st_size, "mtime_ns": p.stat().st_mtime_ns}
            for bag in bags for p in sorted(bag.iterdir())
            if p.suffix == ".db3" or p.name == "metadata.yaml"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=os.environ.get("CALIB_ROOT"))
    parser.add_argument("--input-root", type=Path,
                        help="Optional existing PointCloud2 bag directory")
    args = parser.parse_args()
    if args.root is None:
        parser.error("Source scripts/calibration_env.sh or pass --root")
    root = args.root.resolve()
    input_root = args.input_root.resolve() if args.input_root else root / "bags"
    names = [f"pose_{i:02d}" for i in range(1, 6)]
    bags = [input_root / name for name in names]
    lidar_types = set()
    for bag in bags:
        info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        topics = {t["topic_metadata"]["name"]: t for t in info["topics_with_message_count"]}
        for name, expected in (
            ("/camera/camera/color/image_raw", "sensor_msgs/msg/Image"),
            ("/camera/camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
        ):
            if topics[name]["topic_metadata"]["type"] != expected or topics[name]["message_count"] < 1:
                raise RuntimeError(f"Invalid or empty {name} in {bag}")
        if topics["/livox/lidar"]["message_count"] < 1:
            raise RuntimeError(f"Empty lidar input: {bag}")
        lidar_types.add(topics["/livox/lidar"]["topic_metadata"]["type"])
    if not lidar_types.issubset({"livox_ros_driver2/msg/CustomMsg", "sensor_msgs/msg/PointCloud2"}):
        raise RuntimeError(f"Unsupported lidar types: {lidar_types}")
    original_snapshot = snapshot(bags)

    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    if "livox_ros_driver2/msg/CustomMsg" in lidar_types:
        converted = fresh_path(root, "bags_points")
        subprocess.run([sys.executable, str(Path(__file__).with_name("convert_calibration_bags.py")),
                        str(input_root), str(converted)], env=env, check=True)
        input_root = converted

    stage = fresh_path(root, "preprocessed_build")
    stage.mkdir()
    image = os.environ.get("CALIB_IMAGE", "koide3/direct_visual_lidar_calibration:humble")
    image_id = subprocess.check_output(
        ["docker", "image", "inspect", image, "--format", "{{.Id}}"], text=True
    ).strip()
    cmd = ["docker", "run", "--rm", "--cpus=1", "--memory=5g", "--memory-swap=5g",
           "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1",
           "-v", f"{input_root}:/input:ro", "-v", f"{stage}:/output", image,
           "ros2", "run", "direct_visual_lidar_calibration", "preprocess", "/input", "/output",
           "--image_topic", "/camera/camera/color/image_raw",
           "--camera_info_topic", "/camera/camera/color/camera_info",
           "--points_topic", "/livox/lidar", "--intensity_channel", "intensity", "--min_distance", "0.5"]
    log_path = root / f"{stage.name}.log"
    print(f"Processing fresh output: {stage}\nLog: {log_path}", flush=True)
    with log_path.open("w") as log:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        for line in proc.stdout:
            print(line, end="", flush=True)
            log.write(line)
        if proc.wait() != 0:
            raise RuntimeError(f"Preprocessing failed; original results preserved. See {log_path}")
    data = json.loads((stage / "calib.json").read_text())
    if sorted(data["meta"]["bag_names"]) != names or data.get("results"):
        raise RuntimeError("Output dataset or fresh calibration state is invalid")
    for name in names:
        for suffix in (".png", ".ply", "_lidar_indices.png", "_lidar_intensities.png"):
            p = stage / f"{name}{suffix}"
            if not p.is_file() or p.stat().st_size == 0:
                raise RuntimeError(f"Incomplete output: {p}")
        with (stage / f"{name}.ply").open("rb") as ply:
            count = None
            for _ in range(100):
                line = ply.readline().decode("ascii").strip()
                if line.startswith("element vertex "):
                    count = int(line.split()[-1])
                if line == "end_header":
                    break
            if count is None or count <= 0:
                raise RuntimeError(f"No points in {name}")
    if snapshot(bags) != original_snapshot:
        raise RuntimeError("Input files changed during processing; results not promoted")
    provenance = {"input_root": str(input_root), "source_files": original_snapshot,
                  "solver_image_id": image_id, "completed_at": datetime.now().astimezone().isoformat()}
    # Stage files belong to root inside Docker; its parent remains user-owned.
    # Store provenance beside the stage before moving its files into place.
    manifest = root / f"{stage.name}_sources.json"
    manifest.write_text(json.dumps(provenance, indent=2))
    current = root / "preprocessed"
    if current.exists():
        backup = fresh_path(root, "preprocessed_archive")
        current.rename(backup)
        old_manifest = root / "preprocessed_sources.json"
        if old_manifest.exists():
            old_manifest.rename(backup / "SOURCE_BAGS.json")
        print(f"Archived previous output: {backup}", flush=True)
    stage.rename(current)
    manifest.rename(root / "preprocessed_sources.json")
    config = Path(__file__).resolve().parents[1] / "src/FAST-LIVO2/config/mid360s_d435i_calibrated.yaml"
    if config.exists():
        old_config = fresh_path(root, "old_visual_config").with_suffix(".yaml")
        # fresh_path checks the base name; also guard the full filename.
        if old_config.exists():
            old_config = root / f"old_visual_config_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.yaml"
        config.rename(old_config)
        print(f"Archived old visual config: {old_config}", flush=True)
    print(f"Verified and activated five NEW datasets: {current}", flush=True)
    print("Close any old calibration GUI, then restart initial_guess_manual. New extrinsics must be estimated.", flush=True)


if __name__ == "__main__":
    main()
