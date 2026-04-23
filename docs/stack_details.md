# Stack Details

This file describes the current active perception stack in this repository.

## Active Bringup

Use [av_stack_bringup.launch.py](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/visualization/launch/av_stack_bringup.launch.py).

The default stack parameter file is
[ekf_tracker_stack.yaml](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/visualization/config/ekf_tracker_stack.yaml).

## Active Pipeline

`KITTI replay -> lidar_processing -> /lidar_detections + /lidar_detection_markers`

`KITTI replay -> camera_processing -> /camera_stereo_detections + /camera_stereo_detection_markers`

`/lidar_detections + /camera_stereo_detections -> ekf_multi_object_tracker -> /tracked_objects + /tracked_markers`

## Package Roles

- `lidar_processing`
  Produces LiDAR 3D detections and LiDAR debug markers.
- `camera_processing`
  Produces stereo-based 3D detections and camera debug markers.
- `fusion_core`
  Owns temporal tracking through `ekf_multi_object_tracker`.
- `visualization`
  Owns launch files and stack-level configuration.

## Tracker Summary

The active tracker is
[ekf_multi_object_tracker.cpp](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/fusion_core/src/ekf_multi_object_tracker.cpp).

Current behavior:

- tracking frame is `map`
- EKF state is `[px, py, vx, vy, length, width]`
- LiDAR is the primary source for tracked object footprint
- stereo updates position only in normal operation
- stereo-only births use conservative fallback dimensions until LiDAR support arrives
- LiDAR updates can refresh planar yaw, which is smoothed before publishing
- lifecycle is probability-based with confirmation and deletion thresholds

## Outputs

Primary tracker outputs:

- `/tracked_objects`
- `/tracked_markers`

Useful sensor-side debug topics:

- `/lidar_detection_markers`
- `/camera_stereo_detection_markers`
- `/processed_lidar_pc`

## Legacy Path

The older `tracking_based_fusion` path is kept in the repository as legacy reference code.
It is no longer the default stack bringup.
