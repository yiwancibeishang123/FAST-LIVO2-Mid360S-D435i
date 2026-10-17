#!/usr/bin/env python3
"""Copy static calibration bags, converting Livox CustomMsg to XYZ/intensity.

This conversion is for static camera calibration only, not FAST-LIVO2 input:
per-point scan times, tags and scan lines are intentionally omitted.
Original bags are never modified. Other topics and bag timestamps are copied.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def to_points(msg):
    n = len(msg.points)
    if msg.point_num != n:
        raise ValueError("CustomMsg point_num does not match its points array")
    values = np.fromiter(
        (value for p in msg.points for value in (p.x, p.y, p.z, float(p.reflectivity))),
        dtype="<f4", count=n * 4,
    ).reshape(n, 4)
    cloud = PointCloud2()
    cloud.header = msg.header
    cloud.height = 1
    cloud.width = n
    cloud.fields = [
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z", "intensity"))
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = n * 16
    cloud.is_dense = bool(np.isfinite(values[:, :3]).all())
    cloud.data = values.tobytes()
    return cloud


def convert(src, dst):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(src), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topics = reader.get_all_topics_and_types()
    lidar = next((t for t in topics if t.name == "/livox/lidar"), None)
    if lidar is None or lidar.type not in (
        "livox_ros_driver2/msg/CustomMsg", "sensor_msgs/msg/PointCloud2"
    ):
        raise ValueError(f"Unsupported lidar topic in {src}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(dst), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    for topic in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic.name,
            type="sensor_msgs/msg/PointCloud2" if topic.name == lidar.name else topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles,
        ))

    counts = {}
    lidar_frames = 0
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == lidar.name:
            if lidar.type == "livox_ros_driver2/msg/CustomMsg":
                payload = serialize_message(to_points(deserialize_message(payload, CustomMsg)))
            lidar_frames += 1
        writer.write(topic, payload, timestamp)
    del writer  # Flush SQLite and write metadata.yaml before checking counts.
    del reader

    metadata = yaml.safe_load((dst / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    saved = {t["topic_metadata"]["name"]: t["message_count"]
             for t in metadata["topics_with_message_count"]}
    if saved != counts:
        raise RuntimeError(f"Message counts changed while converting {src}")
    expected = yaml.safe_load((src / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    original = {t["topic_metadata"]["name"]: t["message_count"]
                for t in expected["topics_with_message_count"]}
    if original != saved:
        raise RuntimeError(f"Output is incomplete for {src}")
    provenance = {
        "source": str(src.resolve()), "source_lidar_type": lidar.type,
        "message_counts": counts,
        "source_sha256": {p.name: file_hash(p) for p in sorted(src.iterdir())
                          if p.suffix == ".db3" or p.name == "metadata.yaml"},
        "conversion": "Static calibration XYZ/intensity only; header and bag timestamps preserved",
    }
    (dst / "conversion.json").write_text(json.dumps(provenance, indent=2))
    print(f"Verified {src.name}: {lidar_frames} PointCloud2 frames, all topic counts preserved", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_root", type=Path)
    parser.add_argument("output_root", type=Path)
    args = parser.parse_args()
    sources = [args.input_root / f"pose_{i:02d}" for i in range(1, 6)]
    for src in sources:
        if not (src / "metadata.yaml").is_file():
            parser.error(f"Missing bag: {src}")
    if args.output_root.exists():
        parser.error("Choose a new output directory; existing output is never overwritten")
    args.output_root.mkdir(parents=True)
    for src in sources:
        convert(src, args.output_root / src.name)


if __name__ == "__main__":
    main()
