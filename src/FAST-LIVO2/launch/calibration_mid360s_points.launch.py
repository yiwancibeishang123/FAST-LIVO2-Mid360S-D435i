"""Run the existing Mid-360S driver as PointCloud2 for camera calibration only.

Stop the CustomMsg driver before starting this launch. FAST-LIVO2 mapping uses
the original CustomMsg launch and must not be started with this one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = get_package_share_directory("livox_ros_driver2")
    config = os.path.join(driver_share, "config", "MID360s_config.json")
    if not os.path.isfile(config):
        raise RuntimeError(f"Mid-360S driver configuration is missing: {config}")

    return LaunchDescription([
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 0,  # /livox/lidar: sensor_msgs/msg/PointCloud2
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
    ])
