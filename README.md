# ROS2 C++ Sensor Fusion Mini-Stack for Autonomous Driving


---

## 1. Objective

Build a compact, reproducible ROS2 C++ portfolio project that demonstrates:

- ROS2 node architecture and separation of concerns
- Modern C++ ownership and interface design
- Camera + LiDAR ingestion, late/object-level fusion, and tracking
- Behavior-level safety decision output (GO/SLOW/STOP)
- Practical DDS/QoS reasoning
- Disciplined Git workflow with tests and CI

---

## 2. Scope

### In scope

- ROS2 C++ packages and modules
- Small curated KITTI subset as replay source
- Custom KITTI replay adapter (avoid starting with rosbag conversion)
- Camera + LiDAR input handling
- Late/object-level fusion and basic tracking core
- Simple behavior-level safety decision logic
- Core unit tests + CI
- Documentation and clear Git history

### Out of scope

- Full KITTI ingestion framework
- Detector training or full perception model development
- Raw end-to-end point cloud object detection
- Advanced behavior planning or full trajectory optimization
- Vehicle control stack, SLAM/localization stack
- Production-level real-time optimization
- Rosbag conversion as required for v1

---

## 3. Data strategy

- Use a small, curated KITTI subset (camera, LiDAR, optional pose/odometry)
- KITTI is replayed, not treated as the core challenge
- Rule: do not start with rosbag conversion; prove pipeline with custom replay adapter first

---

## 4. Deliverables

### Mandatory output

- Fused tracked object list
- Safety-aware decision state: `GO`, `SLOW`, `STOP`

### Optional output (v1 optional)

- Local target path or trajectory marker (not required in v1)

### Core pipeline

`KITTI replay -> sensor streams -> fusion/tracking -> behavior decision -> visualization/logging`

---

## 5. Reproducibility criteria

Another developer should be able to:

1. Build the project
2. Run the stack
3. Replay a small scenario
4. Observe fused object output and behavior decisions

---

## 6. Project goals

### Goal 1 — technical

- Clear ROS2 architecture with core logic in libraries and light ROS wrappers.

### Goal 2 — C++ quality

- Explicit ownership, smart pointers, interfaces, and object lifetime handling
- Copy vs move semantics where relevant

### Goal 3 — system understanding

- Topic/message flow, node boundaries, replay pipeline, DDS/QoS design

### Goal 4 — portfolio

- Clean repo with README, docs, tests, CI, demo, meaningful commit history

### Goal 5 — scope discipline

- Finish a working reduction in 3 weeks without scope creep into dataset/advanced planning

---

## 7. Success criteria

### Functional

- Selected KITTI scenario replays pass
- Camera and LiDAR data ingest works
- Fused tracked objects are produced
- Safety decision state updates with scene state
- End-to-end launch reproduces expected behavior

### Engineering

- Thin ROS wrappers and separate core logic
- Meaningful feature branches and commits
- Unit tests for fusion/tracking core
- CI passes
- Architecture/design notes are written

### Learning

Document:

- Why object-level fusion is chosen
- Why trajectory generation is reduced
- Ownership decisions (`shared_ptr` vs `unique_ptr`)
- QoS assumptions and design
- What was intentionally cut for v1

---

## 8. Risks and mitigations

- Dataset overhead: small KITTI subset + replay first; postpone rosbag
- Planning scope explosion: output is decision state not full planner
- Weak C++ focus: core fusion/tracking libraries and documented ownership
- DDS/ROS2 superficiality: document QoS and callback/node design
- Git cosmetic history: branch-per-feature + coherent commits
- Trying to do too much: strict v1 and cut non-essential features
