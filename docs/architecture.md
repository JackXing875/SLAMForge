# LiteVO Architecture

> Industrial-Grade Monocular Visual SLAM System

## Overview

LiteVO is a feature-based monocular SLAM system inspired by ORB-SLAM3. It estimates
6-DoF camera poses and builds a sparse 3D map in real-time using ORB features.

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        INPUT: Monocular Video                    │
└────────────────────────────┬─────────────────────────────────────┘
                             │
                  ┌──────────▼──────────┐
                  │   TRACKING THREAD    │
                  │   (Real-time, ~30ms) │
                  │                      │
                  │ ┌──────────────────┐ │
                  │ │  ORB Extractor   │ │
                  │ └────────┬─────────┘ │
                  │          │           │
                  │ ┌────────▼─────────┐ │
                  │ │  Init / Reloc    │ │
                  │ └────────┬─────────┘ │
                  │          │           │
                  │ ┌────────▼─────────┐ │
                  │ │  Pose Estimation │ │
                  │ └────────┬─────────┘ │
                  │          │           │
                  │ ┌────────▼─────────┐ │
                  │ │  Keyframe Dec.   │ │
                  │ └──────────────────┘ │
                  └──────────┬───────────┘
                             │ new KeyFrame
                  ┌──────────▼───────────┐
                  │  LOCAL MAPPING THREAD │
                  │  (~100-300ms per KF)  │
                  │                       │
                  │ ┌───────────────────┐ │
                  │ │ Map Point Creation │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Local BA (Ceres)  │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Point/KF Culling  │ │
                  │ └───────────────────┘ │
                  └──────────┬────────────┘
                             │
                  ┌──────────▼───────────┐
                  │  LOOP CLOSING THREAD  │
                  │  (asynchronous)       │
                  │                       │
                  │ ┌───────────────────┐ │
                  │ │ DBoW2 Detection   │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Geometry Verify   │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Pose Graph (g2o)  │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Global BA         │ │
                  │ └───────────────────┘ │
                  └───────────────────────┘
```

## Data Structures

### Atlas (Map)
The central data store shared across all threads. Contains:
- **KeyFrames**: all keyframes with their poses, features, and connections
- **MapPoints**: all 3D points with their world positions and observations
- **Covisibility Graph**: weighted graph connecting keyframes that share map points
- **Spanning Tree**: minimal connected subgraph for efficient pose propagation

### KeyFrame
```
KeyFrame
├── FrameId (unique identifier)
├── SE3 pose (world-to-camera transform)
├── std::vector<Feature> (ORB keypoints + descriptors)
├── std::vector<MapPointId> (observed 3D points)
├── std::set<FrameId> (covisibility neighbors)
└── FrameId (spanning tree parent)
```

### MapPoint
```
MapPoint
├── MapPointId (unique identifier)
├── Vec3 (world position)
├── Vec3 (mean viewing direction)
├── std::map<FrameId, int> (observations: keyframe → feature index)
├── int (observation count)
└── double (max descriptor distance)
```

## Thread Safety

The Atlas uses `std::shared_mutex` for read-write locking:
- Tracking holds a **shared lock** during frame processing (read-mostly)
- Local Mapping holds a **unique lock** when adding map points or keyframes
- Loop Closing holds a **unique lock** during map corrections

Keyframes and map points use `std::mutex` for fine-grained locking on individual elements.

## Configuration

All runtime parameters are loaded from YAML files at startup. See `config/` directory
for dataset-specific presets (KITTI, EuRoC, TUM).

## Dependencies

| Library | Role |
|---------|------|
| Eigen3 | Linear algebra (vectors, matrices) |
| Sophus | SE(3) Lie group operations |
| OpenCV 4.x | Image I/O, ORB features, basic geometry |
| Ceres Solver | Local & global bundle adjustment |
| g2o | Pose graph optimization |
| FBOW / DBoW2 | Visual bag-of-words for loop detection |
| spdlog | Structured logging |
| yaml-cpp | Configuration file parsing |
| CLI11 | Command-line interface |

## Performance Targets

| Component | Target |
|-----------|--------|
| Feature extraction | < 10ms (1200 ORB) |
| Tracking (per frame) | < 30ms total |
| Local BA (20 KFs) | < 300ms |
| Loop detection | < 50ms per query |
| Pose graph optimization | < 500ms |
