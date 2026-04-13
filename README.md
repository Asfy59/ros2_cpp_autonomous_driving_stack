# ROS2 C++ Perception and Sensor Fusion Mini-Stack for Autonomous Driving

## Objective

This repository is a ROS2 C++ autonomous driving mini-stack built around KITTI replay.

The goal is to build a compact, readable portfolio project that demonstrates:

- ROS2 node/package architecture
- modern C++ ownership and pipeline design
- LiDAR and camera ingestion from KITTI replay
- perception-oriented preprocessing and detection
- a clear path toward sensor fusion and tracked object outputs

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
  - can optionally publish an overlay image for visual debugging
  - supports interval-based runtime metrics and CSV logging
- KITTI replay infrastructure from the `ros2_kitti_*` packages
- URDF/TF and RViz visualization support
- optional odometry integration from the `ros2_kitti` stack

Still in progress:

- fusion/tracking
- object-level fused outputs
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

My project-specific work in this repository is focused on the custom processing and fusion layer, including packages such as:

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

I treat these packages as the infrastructure layer rather than the main portfolio focus.

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
- `fusion_core`: early scaffold
- `replay_adapter`: not required yet

## Camera Processing

The camera-side perception layer is implemented as a custom `camera_processing` package rather than a direct wrapper around an external detector repository.

Current implementation:

- uses a single monocular camera input, currently `p2_img`
- keeps the first version monocular rather than stereo
- handles image subscription, preprocessing, and camera-side detection in ROS2/C++
- publish `vision_msgs/Detection2DArray` for downstream fusion
- can optionally publish an overlay image with rendered detections for RViz debugging

This keeps the camera stack aligned with the rest of the project:

- `lidar_processing` owns LiDAR-side preprocessing
- `camera_processing` owns camera-side preprocessing and 2D detections
- `fusion_core` will own cross-sensor fusion and tracking

The intended sensor responsibilities are:

- `lidar_processing`: spatial filtering, ground removal, clustering, and 3D object candidate generation
- `camera_processing`: semantic 2D object detection
- `fusion_core`: 2D-3D association and fused object generation

## Current Pipeline

Current end-to-end flow:

`KITTI replay -> /lidar_pc -> lidar_processing -> /lidar_detections + /lidar_detection_markers + optional /processed_lidar_pc`

`KITTI replay -> /p2_img -> camera_processing -> /object_detections + optional /overlay_image`

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

![KITTI dataset reference](docs/kitti_dataset.png)

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

![Camera object detection overlay](docs/Object_detection.png)

## LiDAR Processing Visuals

Current before/after visualization for the preprocessing stage:

- raw replayed LiDAR point cloud on `/lidar_pc`
- processed LiDAR point cloud on `/processed_lidar_pc` when enabled
- 3D bounding-box markers on `/lidar_detection_markers`
- object detections on `/lidar_detections`
- side-by-side comparison showing the effect of ROI cropping, voxelization, and optional ground removal

![Raw vs processed LiDAR point cloud](docs/lidar_raw_vs_processed.png)

This should make the `lidar_processing` package easier to evaluate as a standalone perception module.

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

## Runtime Profiling

Both perception nodes now support interval-based profiling aimed at measuring real pipeline cost rather than just one-off debug timings.

Current profiling coverage:

- `camera_processing`: buffer age, conversion, inference, publish, overlay, frame total, average detections
- `lidar_processing`: buffer age, conversion, crop box, voxelization, ground segmentation, clustering, cluster filtering, bounding box estimation, detection conversion, marker conversion, publish stages, frame total, input/output point counts, cluster counts, detection counts

CSV logging behavior:

- disabled by default
- one CSV file per node run
- one row written per `profiling_interval_frames`
- logs stored under `csv_logs/camera_processing/` and `csv_logs/lidar_processing/`
- file names include the node name, KITTI sequence, and UTC timestamp

The profiling work is intended to support later optimization of:

- sensor-specific perception stages inside `camera_processing` and `lidar_processing`
- association and fusion latency inside `fusion_core`

## Running The Current Stack

Build:

```bash
colcon build --packages-select auto_stack_msgs lidar_processing camera_processing visualization
source install/setup.bash
```

Launch:

```bash
ros2 launch visualization av_stack_bringup.launch.py dataset_path:=/path/to/kitti_dataset dataset_number:=0
```

Enable CSV profiling logs:

```bash
ros2 launch visualization av_stack_bringup.launch.py \
  dataset_path:=/path/to/kitti_dataset \
  dataset_number:=0 \
  enable_camera_csv_logging:=true \
  enable_lidar_csv_logging:=true
```

Current LiDAR topics of interest:

- `/lidar_pc`
- `/processed_lidar_pc` when `publish_processed_lidar_pc:=true`
- `/lidar_detections`
- `/lidar_detection_markers`

Current camera topics of interest:

- `/p2_img`
- `/object_detections`
- `/p2_camera_info` when enabled

## Attribution

This project builds on and integrates the `ros2_kitti_*` replay/visualization stack from:

- `tengfoonglam/kitti_odometry_replayer_ros2`
- https://github.com/tengfoonglam/kitti_odometry_replayer_ros2

This repository uses that upstream project for the KITTI replay, URDF/TF, RViz, message, and odometry infrastructure.

![ros2_kitti upstream reference](docs/ros2_kitti.png)

## Development Notes

- I use `ros2_kitti_*` as the base/integration layer
- I add custom work on top of that layer rather than deeply refactoring the upstream stack
- the main app entrypoint is the custom bringup launch in `visualization`
- the current portfolio focus is perception and sensor fusion, not end-to-end AV behavior planning

## Roadmap

- [x] Integrate KITTI replay and visualization infrastructure
- [x] Add custom LiDAR preprocessing node
- [x] Add LiDAR preprocessing parameters in bringup
- [x] Add a basic LiDAR raw-vs-processed comparison script
- [x] Capture and document LiDAR preprocessing visuals
- [x] Add camera processing node
- [x] Add ONNX-based 2D camera detections
- [x] Add optional camera overlay visualization for RViz2
- [x] Add interval runtime profiling for camera and LiDAR
- [x] Add CSV export for camera and LiDAR profiling
- [x] Separate custom stack messages into `auto_stack_msgs`
- [ ] Decide and document the v1 fusion strategy
- [ ] Implement fusion and tracking in `fusion_core`
- [ ] Publish tracked object outputs
- [ ] Evaluate LiDAR-side clustering/detection vs fusion on current outputs
- [ ] Optimize the perception and fusion pipeline using the profiling data
- [ ] Fine-tune the YOLO model for autonomous-driving-relevant classes
- [ ] Improve automated tests
- [ ] Add CI and polish documentation
- [ ] Revisit behavior planning only after the perception and fusion stack is stable
