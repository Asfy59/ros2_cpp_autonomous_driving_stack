# ROS2 C++ Sensor Fusion Mini-Stack for Autonomous Driving

## Objective

This repository is a ROS2 C++ autonomous driving mini-stack built around KITTI replay.

The goal is to build a compact, readable portfolio project that demonstrates:

- ROS2 node/package architecture
- modern C++ ownership and pipeline design
- LiDAR and camera ingestion from KITTI replay
- object-level fusion and tracking
- behavior-level output such as `GO`, `SLOW`, `STOP`

## Current Status

The project currently has:

- KITTI replay infrastructure from the `ros2_kitti_*` packages
- URDF/TF and RViz visualization support
- optional odometry integration from the `ros2_kitti` stack
- a custom `lidar_processing` node that:
  - subscribes to replayed LiDAR data
  - crops the ROI
  - voxelizes the cloud
  - optionally removes the ground plane with RANSAC
  - publishes a processed LiDAR point cloud

Still in progress:

- camera processing
- fusion/tracking
- behavior decision output
- tests/CI beyond the current helper scripts and existing upstream tests

## Repo Organization

This repo contains two kinds of packages.

## Attribution

This project builds on and integrates the `ros2_kitti_*` replay/visualization stack from:

- `tengfoonglam/kitti_odometry_replayer_ros2`
- https://github.com/tengfoonglam/kitti_odometry_replayer_ros2

This repository builds on that upstream project for the KITTI replay, URDF/TF, RViz, message, and odometry infrastructure.

![ros2_kitti upstream reference](docs/ros2_kitti.png)

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
- `visualization`: owns app-level bringup launch
- `fusion_core`: early scaffold
- `camera_processing`: scaffold only
- `replay_adapter`: not required yet

## Planned Camera Processing

The camera-side perception layer is planned as a custom `camera_processing` package rather than a direct wrapper around an external detector repository.

Current direction:

- start with a single camera input, using `p2_img`
- keep the first version monocular rather than stereo
- handle image subscription, preprocessing, and camera-side candidate extraction in ROS2/C++
- publish a clean camera-side output that can later be consumed by `fusion_core`

This keeps the camera stack aligned with the rest of the project:

- `lidar_processing` owns LiDAR-side preprocessing
- `camera_processing` will own camera-side preprocessing
- `fusion_core` will own cross-sensor fusion, tracking, and decision logic

## Current Pipeline

Current end-to-end flow:

`KITTI replay -> /lidar_pc -> lidar_processing -> /processed_lidar_pc -> RViz`

Planned later flow:

`KITTI replay -> camera + lidar -> processing -> fusion/tracking -> decision -> visualization`

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

## LiDAR Processing Node

The `lidar_processing` node currently performs:

1. latest-message buffering outside the callback
2. ROS `PointCloud2` to PCL conversion
3. ROI crop with `pcl::CropBox`
4. voxel downsampling with `pcl::VoxelGrid`
5. optional ground removal with `pcl::SACSegmentation` and `pcl::ExtractIndices`
6. publication of `/processed_lidar_pc`

Current configurable parameters:

- `processing_rate`
- `crop_box_min`
- `crop_box_max`
- `voxel_leaf_size`
- `enable_ground_segmentation`

These are configured in the app bringup launch:

- [av_stack_bringup.launch.py](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/visualization/launch/av_stack_bringup.launch.py)

## LiDAR Processing Visuals

Current before/after visualization for the preprocessing stage:

- raw replayed LiDAR point cloud on `/lidar_pc`
- processed LiDAR point cloud on `/processed_lidar_pc`
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

## Running The Current Stack

Build:

```bash
colcon build --packages-select lidar_processing visualization
source install/setup.bash
```

Launch:

```bash
ros2 launch visualization av_stack_bringup.launch.py dataset_path:=/path/to/kitti_dataset dataset_number:=0
```

Current LiDAR topics of interest:

- `/lidar_pc`
- `/processed_lidar_pc`

## Development Notes

- I use `ros2_kitti_*` as the base/integration layer
- I add custom work on top of that layer rather than deeply refactoring the upstream stack
- the main app entrypoint is the custom bringup launch in `visualization`

## Roadmap

- [x] Integrate KITTI replay and visualization infrastructure
- [x] Add custom LiDAR preprocessing node
- [x] Add LiDAR preprocessing parameters in bringup
- [x] Add a basic LiDAR raw-vs-processed comparison script
- [x] Capture and document LiDAR preprocessing visuals
- [ ] Add camera processing node
- [ ] Implement fusion and tracking in `fusion_core`
- [ ] Publish tracked object outputs
- [ ] Publish decision output such as `GO`, `SLOW`, `STOP`
- [ ] Improve automated tests
- [ ] Add CI and polish documentation
