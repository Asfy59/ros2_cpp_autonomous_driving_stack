# V1 Stack Contract

This document records the current interface for the active tracker stack.

## Active Public Topics

- `/tracked_objects`
- `/tracked_markers`

## Supporting Detection Topics

- `/lidar_detections`
- `/lidar_detection_markers`
- `/camera_stereo_detections`
- `/camera_stereo_detection_markers`

## Frame Convention

- Tracker state is maintained in `map`
- Sensor detections are transformed into `map` before association and update

## Ownership

- `lidar_processing`
  LiDAR detections only
- `camera_processing`
  Stereo detections only
- `fusion_core`
  Persistent tracking and fused tracked-object output
- `visualization`
  Bringup and parameterization

## Current Status

The active tracker implementation is
[ekf_multi_object_tracker.cpp](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/fusion_core/src/ekf_multi_object_tracker.cpp).

`decision_state` is not part of the active EKF tracker bringup.
