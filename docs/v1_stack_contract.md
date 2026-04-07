# V1 Stack Contract

## Public v1 outputs

- `/tracked_objects` (`ros2_kitti_msgs/msg/TrackedObjectArray`)
- `/decision_state` (`ros2_kitti_msgs/msg/DecisionState`)

These are the two public outputs that define "working v1" for the mini-stack.

## Input source

- Replay source: `ros2_kitti_replay`
- Canonical replay topics:
  - `/lidar_pc`
  - `/p0_img`
  - `/p1_img`
  - `/p2_img`
  - `/p3_img`
  - `/clock`

## Package ownership

- `ros2_kitti_msgs`: shared interfaces only
- `replay_adapter`: optional normalization layer for topics, frames, and QoS
- `lidar_processing`: LiDAR-side object proposal extraction
- `camera_processing`: camera-side detections or enrichment
- `fusion_core`: tracking, fusion, and behavior decision logic
- `visualization`: top-level bringup and RViz entrypoints
