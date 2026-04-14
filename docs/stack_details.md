# Stack Details

This document keeps the longer implementation notes for the stack. The repo
root `README.md` stays intentionally high level.

## Current Status

The project currently has:

- a custom `lidar_processing` node that:
  - subscribes to replayed LiDAR data
  - crops the ROI
  - voxelizes the cloud
  - optionally removes the ground plane with RANSAC
  - clusters and filters obstacle candidates
  - publishes `vision_msgs/Detection3DArray`
  - publishes RViz-friendly 3D bounding box markers
  - can optionally publish a processed LiDAR point cloud for debugging
  - supports interval-based runtime metrics and CSV logging
- a custom `camera_processing` node that:
  - subscribes to replayed monocular camera data
  - runs ONNX Runtime-based YOLO inference
  - publishes `vision_msgs/Detection2DArray`
  - can optionally publish `sensor_msgs/CameraInfo` derived from KITTI calibration
  - can optionally publish an overlay image for visual debugging
  - supports interval-based runtime metrics and CSV logging
- a custom `fusion_core` node that:
  - subscribes to LiDAR 3D detections, camera 2D detections, and camera calibration
  - projects LiDAR 3D boxes into the `p2` image plane using TF + camera intrinsics
  - associates 3D LiDAR proposals with 2D camera detections
  - publishes first-pass `TrackedObjectArray` and `DecisionState` outputs
  - supports interval-based runtime metrics and CSV logging
- KITTI replay infrastructure from the `ros2_kitti_*` packages
- URDF/TF and RViz visualization support
- optional odometry integration from the `ros2_kitti` stack

Still in progress:

- persistent tracking / velocity estimation
- deeper fusion evaluation and optimization
- tests/CI beyond the current helper scripts and existing upstream tests

Current project focus:

- perception quality
- sensor alignment
- fusion pipeline design and implementation
- runtime profiling and optimization

Sensor-role design principle:

- LiDAR is treated as the primary source for geometry, range, and 3D object proposals
- camera is treated as the primary source for semantics, class identity, and appearance cues
- fusion is responsible for association and confidence consolidation, not raw sensor-heavy preprocessing

Explicitly deprioritized for now:

- behavior planning / decision making

## Repo Organization

This repo contains two kinds of packages.

Project-specific work in this repository is focused on the custom processing and
fusion layer, including packages such as:

- `lidar_processing`
- `camera_processing`
- `fusion_core`
- `visualization`

### Integrated / upstream-style infrastructure

These provide the replay, TF, description, RViz, and odometry base:

- `ros2_kitti_core`
- `ros2_kitti_replay`
- `ros2_kitti_description`
- `ros2_kitti_msgs`
- `ros2_kitti_odom`
- `ros2_kitti_odom_kiss_icp`
- `ros2_kitti_odom_open3d`
- `ros2_kitti_rviz_plugin`

These are treated as the infrastructure layer rather than the main portfolio focus.

### Custom project packages

These are the packages where the project-specific work is being implemented:

- `lidar_processing`
- `camera_processing`
- `fusion_core`
- `replay_adapter`
- `visualization`

Current custom progress:

- `lidar_processing`: active and working
- `camera_processing`: active and publishing detections
- `visualization`: owns app-level bringup launch
- `fusion_core`: first-pass fusion and decision outputs implemented
- `replay_adapter`: not required yet

## Current Pipeline

Current end-to-end flow:

`KITTI replay -> /lidar_pc -> lidar_processing -> /lidar_detections + /lidar_detection_markers + optional /processed_lidar_pc`

`KITTI replay -> /p2_img -> camera_processing -> /object_detections + optional /p2_camera_info + optional /overlay_image`

`/lidar_detections + /object_detections + /p2_camera_info -> fusion_core -> /tracked_objects + /decision_state`

Planned later flow:

`KITTI replay -> camera + lidar -> processing -> fusion/tracking -> fused outputs -> visualization`

Fusion is planned around complementary sensor strengths rather than duplicate estimation:

- LiDAR provides stable geometry
- camera provides semantic labeling
- fusion assigns semantics to 3D structure

## What KITTI Provides vs What This Repo Adds

### Raw KITTI provides

- timestamps
- LiDAR scans
- camera images
- calibration
- pose / ground-truth trajectory for supported odometry sequences

Dataset reference:

- KITTI Odometry Dataset
- https://www.cvlibs.net/datasets/kitti/eval_odometry.php

![KITTI dataset reference](kitti_dataset.png)

### This repo adds

- ROS2 topics and message publishing
- frame IDs and timestamps on messages
- TF tree and URDF loading
- replay controls and RViz integration
- custom LiDAR preprocessing node
- custom camera detection node
- custom stack message package for fused/tracked outputs

## LiDAR Processing Node

The `lidar_processing` node currently performs:

1. latest-message buffering outside the callback
2. ROS `PointCloud2` to PCL conversion
3. ROI crop with `pcl::CropBox`
4. voxel downsampling with `pcl::VoxelGrid`
5. optional ground removal with `pcl::SACSegmentation` and `pcl::ExtractIndices`
6. Euclidean clustering of non-ground points into obstacle candidates
7. cluster filtering by point count and physical size
8. axis-aligned 3D bounding-box estimation per filtered cluster
9. publication of `vision_msgs/Detection3DArray`
10. publication of `visualization_msgs/MarkerArray` for RViz debugging
11. optional publication of `/processed_lidar_pc`
12. interval-based runtime profiling and optional CSV export

Current configurable parameters:

- `processing_rate`
- `crop_box_min`
- `crop_box_max`
- `voxel_leaf_size`
- `publish_processed_lidar_pc`
- `enable_ground_segmentation`
- `cluster_tolerance_m`
- `min_cluster_points`
- `max_cluster_points`
- `min_cluster_size`
- `max_cluster_size`
- `profiling_interval_frames`
- `enable_csv_logging`
- `csv_log_dir`
- `dataset_sequence`

These are configured in the app bringup launch:

- [av_stack_bringup.launch.py](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/visualization/launch/av_stack_bringup.launch.py)

## Camera Processing Node

The `camera_processing` node currently performs:

1. latest-message buffering outside the callback
2. ROS `sensor_msgs/Image` to OpenCV conversion with `cv_bridge`
3. ONNX Runtime-based YOLO inference
4. publication of `vision_msgs/Detection2DArray`
5. optional publication of `sensor_msgs/CameraInfo` derived from KITTI calibration for downstream fusion
6. optional publication of an overlay image for debugging in RViz2
7. interval-based runtime profiling and optional CSV export

Current configurable parameters:

- `processing_rate`
- `model_path`
- `publish_camera_info`
- `publish_overlay_image`
- `profiling_interval_frames`
- `enable_csv_logging`
- `csv_log_dir`
- `dataset_path`
- `dataset_sequence`
- `camera_name`

Current camera topics of interest:

- `/p2_img`
- `/object_detections`
- `/p2_camera_info` when enabled
- `/overlay_image` when enabled

Current camera-side detection visualization:

- object detections rendered on the monocular input image
- useful for quickly checking detector coverage and label placement before fusion

![Camera object detection overlay](Object_detection.png)

## LiDAR Processing Visuals

Current before/after visualization for the preprocessing stage:

- raw replayed LiDAR point cloud on `/lidar_pc`
- processed LiDAR point cloud on `/processed_lidar_pc` when enabled
- 3D bounding-box markers on `/lidar_detection_markers`
- object detections on `/lidar_detections`
- side-by-side comparison showing the effect of ROI cropping, voxelization, and optional ground removal

![Raw vs processed LiDAR point cloud](lidar_raw_vs_processed.png)

This helps evaluate `lidar_processing` as a standalone perception module.

## TF / URDF / Timestamp Notes

For later fusion work, the important message fields are:

- `header.stamp`
- `header.frame_id`

In the current replay stack:

- `header.stamp` comes from KITTI `times.txt`
- LiDAR `header.frame_id` is set by the replay node to a prefixed LiDAR frame
- camera `header.frame_id` is set by the replay node to prefixed camera frames such as `p2`
- URDF + `robot_state_publisher` define the fixed sensor geometry
- replay / odometry nodes provide motion transforms over time

This is the basis for future sensor alignment and fusion.

## Fusion Core Node

The current `fusion_core` node is a first-pass fusion stage built on top of the processed outputs from the LiDAR and camera nodes.

Current implementation:

1. subscribes to `vision_msgs/Detection3DArray` from `lidar_processing`
2. buffers recent camera `vision_msgs/Detection2DArray` messages
3. subscribes to `sensor_msgs/CameraInfo` from `camera_processing`
4. uses LiDAR detections as the frame trigger for fusion
5. projects each 3D LiDAR box into the `p2` image using TF + camera intrinsics
6. associates projected 2D ROIs against camera detections using IoU and center-distance gating
7. publishes `auto_stack_msgs/TrackedObjectArray` on `/tracked_objects`
8. publishes `auto_stack_msgs/DecisionState` on `/decision_state`
9. supports interval-based runtime profiling and optional CSV export

Current fusion topics of interest:

- `/lidar_detections`
- `/object_detections`
- `/p2_camera_info`
- `/tracked_objects`
- `/decision_state`

Current first-pass design choices:

- LiDAR remains the source of 3D geometry
- camera provides semantic class enrichment when the association is clean
- track IDs are currently frame-local rather than persistent
- decision logic is intentionally simple and based on the nearest forward LiDAR-backed obstacle

## Runtime Profiling

All three custom processing nodes now support interval-based profiling aimed at measuring real pipeline cost rather than just one-off debug timings.

Current profiling coverage:

- `camera_processing`: buffer age, conversion, inference, publish, overlay, frame total, average detections
- `lidar_processing`: buffer age, conversion, crop box, voxelization, ground segmentation, clustering, cluster filtering, bounding box estimation, detection conversion, marker conversion, publish stages, frame total, input/output point counts, cluster counts, detection counts
- `fusion_core`: buffer age, TF lookup, 3D box projection, association, decision, publish, frame total, camera-LiDAR skew, accepted match IoU, input detection counts, matched/unmatched counts, output tracked-object count

CSV logging behavior:

- disabled by default
- one CSV file per node run
- one row written per `profiling_interval_frames`
- logs stored under `csv_logs/camera_processing/`, `csv_logs/lidar_processing/`, and `csv_logs/fusion_core/`
- file names include the node name, KITTI sequence, and UTC timestamp

The profiling work is intended to support later optimization of:

- sensor-specific perception stages inside `camera_processing` and `lidar_processing`
- association and fusion latency inside `fusion_core`

## Attribution

This project builds on and integrates the `ros2_kitti_*` replay/visualization stack from:

- `tengfoonglam/kitti_odometry_replayer_ros2`
- https://github.com/tengfoonglam/kitti_odometry_replayer_ros2

This repository uses that upstream project for the KITTI replay, URDF/TF, RViz, message, and odometry infrastructure.

![ros2_kitti upstream reference](ros2_kitti.png)
