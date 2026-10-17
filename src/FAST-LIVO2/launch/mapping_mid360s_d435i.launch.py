"""Live Mid-360S + D435i launch. Defaults to a low-load LIO-only smoke test."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *_args, **_kwargs):
    share = get_package_share_directory("fast_livo")
    visual_config = LaunchConfiguration("visual_config_file").perform(context).strip()
    config = visual_config or os.path.join(share, "config", "mid360s_d435i_lio.yaml")
    if not os.path.isfile(config):
        raise RuntimeError(f"FAST-LIVO2 parameter file does not exist: {config}")

    with open(config, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["/**"]["ros__parameters"]
    extrinsics = params["extrin_calib"]
    visual = bool(params["common"]["img_en"])
    smoke = LaunchConfiguration("allow_uncalibrated_visual_smoke").perform(context).lower() == "true"
    if visual_config:
        if not visual:
            raise RuntimeError("visual_config_file must enable common.img_en")
        rotation, translation = extrinsics["Rcl"], extrinsics["Pcl"]
        if len(rotation) != 9 or len(translation) != 3:
            raise RuntimeError("Calibrated Rcl (9) and Pcl (3) are required")
        uncalibrated = rotation == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] and translation == [0.0, 0.0, 0.0]
        if uncalibrated and not (smoke and os.path.basename(config) == "mid360s_d435i_visual_smoke.yaml"):
            raise RuntimeError("Uncalibrated Rcl/Pcl require the named visual smoke config and allow_uncalibrated_visual_smoke:=true")
    elif visual:
        raise RuntimeError("Default configuration must remain LIO-only")

    camera_config = os.path.join(share, "config", "camera_d435i_261722072542.yaml")
    rviz_config = os.path.join(share, "rviz_cfg", "fast_livo2.rviz")
    return [
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[config, camera_config],
            output="screen",
            respawn=False,
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("visual_config_file", default_value="", description="Calibrated visual YAML; empty runs LIO only"),
        DeclareLaunchArgument("allow_uncalibrated_visual_smoke", default_value="false", description="Explicitly permit the named smoke config with unknown camera extrinsics"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=_nodes),
    ])
