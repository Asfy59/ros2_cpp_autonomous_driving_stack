# EKF Tracker Validation And Tuning

This guide is for validating `ekf_multi_object_tracker` on replay and tuning the first-pass parameters without changing code.

## Launch

Use the active bringup:

```bash
ros2 launch visualization av_stack_bringup.launch.py \
  dataset_path:=kitti_dataset \
  dataset_number:=00
```

The default parameter file is:

`src/visualization/config/ekf_tracker_stack.yaml`

## Topics To Watch

Core outputs:

- `/tracked_objects`
- `/tracked_markers`
- `/lidar_detections`
- `/camera_stereo_detections`

Useful TF frame:

- `map`

## RViz2 Checks

Set RViz fixed frame to `map`.

Check these first:

1. `lidar_detections` stay attached to real obstacles in the world.
2. `tracked_markers` appear in the same world locations, not moving with the ego vehicle.
3. Confirmed tracks persist across frames instead of blinking on and off.
4. Track ids stay stable for the same object.
5. Parked objects do not develop obviously false velocity arrows or drifting boxes.

If tracked boxes move with ego motion, TF into `map` is wrong.

## Functional Smoke Tests

### Test 1: Birth

Expected:

- a visible LiDAR detection should create a tentative track
- after repeated support it should become confirmed
- a confirmed object should appear on `/tracked_objects`

If tracks never appear:

- check `/lidar_detections`
- check TF availability from detection frame to `map`
- reduce `confirmation_threshold`

### Test 2: Persistence

Expected:

- the same car keeps the same `track_id` over time
- the box should move smoothly, not jump frame-to-frame

If ids keep changing:

- gating is too strict
- process noise may be too small
- LiDAR measurement noise may be too small

### Test 3: Miss And Delete

Expected:

- if an object disappears, its track should decay and eventually vanish
- it should not remain forever

If stale tracks linger too long:

- increase `miss_decay`
- raise `deletion_threshold`

If tracks disappear too fast:

- decrease `miss_decay`
- lower `deletion_threshold`

### Test 4: Stereo Refinement

Expected:

- stereo should slightly stabilize position for supported tracks
- stereo should not directly distort `length` and `width`

If stereo causes unstable position jumps:

- increase `sigma_stereo_px`
- increase `sigma_stereo_py`
- lower `stereo_hit_gain`
- reduce `stereo_gate_mahalanobis_sq`

## Parameter Tuning Guide

### Motion Model

`sigma_a_x`, `sigma_a_y`

- Increase if tracks lag behind moving objects or lose associations during turns or acceleration.
- Decrease if predicted tracks wander too much between measurements.

`sigma_length`, `sigma_width`

- Increase if box size changes are real and the filter is too rigid.
- Decrease if box size jitters too much.

### LiDAR Trust

`sigma_lidar_px`, `sigma_lidar_py`

- Increase if LiDAR position updates look noisy or produce jitter.
- Decrease if LiDAR should pull tracks more strongly.

`sigma_lidar_length`, `sigma_lidar_width`

- Increase if size flickers.
- Decrease if size converges too slowly.

### Stereo Trust

`sigma_stereo_px`, `sigma_stereo_py`

- Increase if stereo pulls tracks to biased or noisy positions.
- Decrease if stereo is useful but has too little effect.

### Birth

`sigma_birth_lidar_*`, `sigma_birth_stereo_*`

- Increase if newborn tracks are overconfident and hard to correct.
- Decrease if newborn tracks are too uncertain and unstable.

`p_init_lidar`, `p_init_stereo`

- Increase if good detections take too long to become confirmed.
- Decrease if too many false tracks become confirmed.

### Association

`lidar_gate_mahalanobis_sq`

- Increase if valid LiDAR matches are rejected.
- Decrease if wrong detections are matching to tracks.

`stereo_gate_mahalanobis_sq`

- Increase if stereo rarely supports tracks.
- Decrease if stereo often supports the wrong track.

### Lifecycle

`lidar_hit_gain`, `stereo_hit_gain`

- Increase if good tracks confirm too slowly.
- Decrease if bad tracks become confirmed too easily.

`miss_decay`

- Increase if ghost tracks remain too long.
- Decrease if tracks die too quickly on temporary occlusion.

`confirmation_threshold`

- Increase to require more evidence before publishing a track.
- Decrease to publish tracks earlier.

`deletion_threshold`

- Increase to remove weak tracks sooner.
- Decrease to keep uncertain tracks alive longer.

## Recommended Tuning Order

1. Verify TF and `map`-frame stability first.
2. Tune LiDAR association and LiDAR update behavior.
3. Tune birth and lifecycle.
4. Only then tune stereo influence.

## Current Limitations

These are known simplifications of the current node:

- classification fusion is simple
- stereo support quality uses Mahalanobis consistency only
- no dedicated automated behavior tests yet
- no decision-state output in this node

Those are not blockers for validating tracking behavior on replay.
