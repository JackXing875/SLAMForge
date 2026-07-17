# SLAMForge Architecture

> Monocular Visual SLAM and Dense Reconstruction

## Overview

SLAMForge is a feature-based monocular SLAM and offline dense-reconstruction system inspired by
ORB-SLAM3. It estimates 6-DoF camera poses and geometric landmarks in real time, then uses the
settled poses and landmarks to reconstruct a colored surface map locally.

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
                  └──────────┬────────────┘
                             │ settled poses + sparse depth anchors
                  ┌──────────▼────────────┐
                  │ DENSE RECONSTRUCTION  │
                  │ (offline, after SLAM) │
                  │                       │
                  │ ┌───────────────────┐ │
                  │ │ Learned Depth    │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Scale Calibration │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Multi-view Filter │ │
                  │ └────────┬──────────┘ │
                  │          │            │
                  │ ┌────────▼──────────┐ │
                  │ │ Colored Fusion   │ │
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

## Data Flow

```
Camera Frame
    │
    ▼
Tracker::Track(image, timestamp)
    │
    ├── Extract ORB features (multi-scale pyramid + quadtree distribution)
    │
    ├── [NOT_INITIALIZED]
    │   └── MonocularInitializer: parallel H/E computation → model selection → initial map
    │
    ├── [OK] TrackWithMotionModel()
    │   ├── Predict pose from constant-velocity model
    │   ├── Project last frame's map points → find matches
    │   └── PnP RANSAC + nonlinear refinement → Tcw
    │
    ├── [OK] TrackLocalMap()
    │   ├── Collect covisible keyframes' map points
    │   ├── Project into current frame → expand 3D-2D matches
    │   └── Motion-only BA → refined Tcw
    │
    ├── NeedNewKeyFrame()?
    │   ├── YES: promote Frame → KeyFrame → Insert to LocalMapper queue
    │   └── NO:  continue
    │
    └── [LOST] Relocalization()
        └── BoW matching against all keyframes → PnP → if inliers > threshold: OK
            │
            ▼ (KeyFrame inserted)
LocalMapper::Run() loop
    │
    ├── ProcessNewKeyFrame()
    │   └── Update covisibility graph + spanning tree
    │
    ├── CreateNewMapPoints()
    │   └── Triangulate with top-N covisible keyframes (parallax + reprojection check)
    │
    ├── CullMapPoints()
    │   └── Remove points with < 25% found ratio in first 3 keyframes
    │
    ├── LocalBundleAdjustment()
    │   └── Ceres: optimize current KF + covisible KFs + their map points (Huber loss)
    │
    └── CullKeyFrames()
        └── Remove KFs where > 90% of map points are seen by ≥ 3 other KFs
            │
            ▼ (Every keyframe also goes to LoopClosing)
LoopClosing::Run() loop
    │
    ├── DetectLoop()
    │   └── Query BoW database → exclude recent KFs → consecutive detection check
    │
    ├── VerifyLoop()
    │   └── BoW-accelerated matching → Sim(3) RANSAC → bidirectional check
    │
    ├── CorrectLoop()
    │   └── Sim(3) propagation → map point fusion → covisibility update
    │
    ├── OptimizePoseGraph()
    │   └── g2o: loop edges + spanning tree edges + covisibility edges → optimize
    │
    └── GlobalBundleAdjustment()  [triggered periodically]
        └── Ceres: all keyframes + all map points (separate thread, can be interrupted)
            │
            ▼ (after tracking and background optimization have stopped)
DenseMapper::Reconstruct()
    │
    ├── SelectKeyFrames()
    │   └── Uniformly sample valid keyframes across the completed route
    │
    ├── InferDepth()
    │   └── Run the bundled Depth Anything V2 Small model through ONNX Runtime
    │
    ├── CalibrateDepth()
    │   └── Robustly fit direct/inverse model depth to SLAM landmarks per keyframe
    │
    ├── CheckMultiViewConsistency()
    │   └── Reject depth unsupported by adjacent calibrated views
    │
    └── FuseVoxels()
        └── Accumulate position, color, and confidence into a deterministic surface cloud
```

## Thread Safety

The Map (Atlas) uses `std::shared_mutex` for read-write locking:
- **Tracking**: holds a **shared lock** during frame processing (read-mostly)
- **Local Mapping**: holds a **unique lock** when adding map points or keyframes
- **Loop Closing**: holds a **unique lock** during map corrections (pose graph + global BA)

Keyframes and map points use individual `std::mutex` for fine-grained locking on specific elements (e.g., MapPoint position updates).

**Lock ordering** (to prevent deadlocks):
1. Map mutex (always first)
2. KeyFrame::GlobalMutex
3. Individual KeyFrame/MapPoint mutexes

## Configuration

All runtime parameters are loaded from YAML files at startup. See `config/` directory
for dataset-specific presets (KITTI, EuRoC, TUM).

## Dependencies

| Library | Role |
|---------|------|
| Eigen3 | Linear algebra (vectors, matrices) |
| Sophus | SE(3) Lie group operations |
| OpenCV 4.x | Image I/O, ORB features, basic geometry |
| ONNX Runtime | CPU inference for the bundled dense-depth model |
| Depth Anything V2 Small | Per-keyframe relative-depth prediction |
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

Dense reconstruction is deliberately outside the real-time tracking path. Its run time depends on
the selected keyframe count, CPU, and output resolution; the desktop UI reports it as a separate
post-processing stage.
