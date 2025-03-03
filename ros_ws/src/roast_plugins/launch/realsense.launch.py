"""realsense launch file"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

configurable_parameters = [
    {"name": "camera_name", "default": "camera"},
    {"name": "serial_no", "default": "''"},
    {"name": "usb_port_id", "default": "''"},
    {"name": "device_type", "default": "''"},
    {"name": "config_file", "default": "''"},
    {"name": "unite_imu_method", "default": "0"},
    {"name": "json_file_path", "default": "''"},
    {"name": "log_level", "default": "info"},
    {"name": "output", "default": "screen"},
    {"name": "depth_module.profile", "default": "0,0,0"},
    {"name": "enable_depth", "default": "true"},
    {"name": "rgb_camera.profile", "default": "0,0,0"},
    {"name": "rgb_camera.enable_auto_exposure", "default": "true"},
    {"name": "enable_color", "default": "true"},
    {"name": "enable_infra1", "default": "false"},
    {"name": "enable_infra2", "default": "false"},
    {"name": "infra_rgb", "default": "false"},
    {"name": "enable_fisheye1", "default": "true"},
    {"name": "enable_fisheye2", "default": "true"},
    {"name": "enable_confidence", "default": "true"},
    {"name": "gyro_fps", "default": "0"},
    {"name": "accel_fps", "default": "0"},
    {"name": "enable_gyro", "default": "false"},
    {"name": "enable_accel", "default": "false"},
    {"name": "enable_pose", "default": "true"},
    {"name": "pose_fps", "default": "200"},
    {"name": "pointcloud.enable", "default": "false"},
    {"name": "pointcloud.stream_filter", "default": "1"},
    {"name": "pointcloud.stream_index_filter", "default": "1"},
    {"name": "enable_sync", "default": "false"},
    {"name": "align_depth.enable", "default": "true"},
    {"name": "colorizer.enable", "default": "false"},
    {"name": "clip_distance", "default": "3"},
    {"name": "linear_accel_cov", "default": "0.01"},
    {"name": "initial_reset", "default": "false"},
    {"name": "allow_no_texture_points", "default": "false"},
    {"name": "pointcloud.ordered_pc", "default": "true"},
    {"name": "publish_tf", "default": "true"},
    {"name": "tf_publish_rate", "default": "0.0"},
    {"name": "diagnostics_period", "default": "0.0"},
    {"name": "decimation_filter.enable", "default": "true"},
    {"name": "rosbag_filename", "default": "''"},
    {"name": "depth_module.exposure.1", "default": "7500"},
    {"name": "depth_module.gain.1", "default": "16"},
    {"name": "depth_module.exposure.2", "default": "1"},
    {"name": "depth_module.gain.2", "default": "16"},
    {"name": "depth_module.exposure", "default": "8500"},
    {"name": "depth_module.hdr_enabled", "default": "false"},
    {"name": "depth_module.enable_auto_exposure", "default": "true"},
    {"name": "hdr_merge.enable", "default": "false"},
    {"name": "wait_for_device_timeout", "default": "-1."},
    {"name": "reconnect_timeout", "default": "6."},
    {"name": "spatial_filter.enable", "default": "true"},
    {"name": "temporal_filter.enable", "default": "true"},
    # {'name': 'decimation_filter.magnitude', 'default': '2'},
    {"name": "spatial_filter.smooth_alpha", "default": "0.5"},
    # {'name': 'spatial_filter.smooth_delta', 'default': '20'},
    # {'name': 'spatial_filter.holes_fill', 'default': '0'},
    # {'name': 'temporal_filter.enable', 'default': 'true'},
    {"name": "temporal_filter.smooth_alpha", "default": "0.2"},
    {"name": "temporal_filter.smooth_delta", "default": "20"},
    # {'name': 'disparity_transform.enable', 'default': 'false'},
]


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(param["name"], default_value=param["default"])
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return {param["name"]: LaunchConfiguration(param["name"]) for param in parameters}


def launch_realsense2_camera(context):
    _config_file = LaunchConfiguration("config_file").perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    node_namespace = LaunchConfiguration("camera_name")
    node_name = LaunchConfiguration("camera_name")
    node_executable = "realsense2_camera_node"
    emulate_tty = False

    return Node(
        package="realsense2_camera",
        namespace=node_namespace,
        name=node_name,
        executable=node_executable,
        emulate_tty=emulate_tty,
        parameters=[
            set_configurable_parameters(configurable_parameters),
            params_from_file,
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters)
        + [OpaqueFunction(function=lambda context: [launch_realsense2_camera(context)])]
    )
