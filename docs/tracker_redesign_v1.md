# Tracker Redesign V1

Archived design note for the transition from `tracking_based_fusion` to
`ekf_multi_object_tracker`.

## Outcome

The redesign was implemented as
[ekf_multi_object_tracker.cpp](/home/asfy/projects/covolv/ros2_av_stack_cpp/src/fusion_core/src/ekf_multi_object_tracker.cpp).

Key decisions that were kept:

- one centralized tracker in `fusion_core`
- tracking in `map`
- EKF state `[px, py, vx, vy, length, width]`
- LiDAR as the primary source for tracked dimensions
- stereo as position support only in normal updates
- probability-based lifecycle with confirmation and deletion

## Status

This file is kept only as a short historical note.
Current behavior and usage are documented in:

- [stack_details.md](/home/asfy/projects/covolv/ros2_av_stack_cpp/docs/stack_details.md)
- [v1_stack_contract.md](/home/asfy/projects/covolv/ros2_av_stack_cpp/docs/v1_stack_contract.md)
- [ekf_tracker_validation.md](/home/asfy/projects/covolv/ros2_av_stack_cpp/docs/ekf_tracker_validation.md)
