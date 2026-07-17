# SLAMForge — Monocular Visual SLAM and Dense Reconstruction

[![License: GPL v3](https://img.shields.io/badge/License-GPL--3.0--only-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue?logo=c%2B%2B)](https://isocpp.org/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-latest-2496ED?logo=docker)](https://github.com/JackXing875/SLAMForge/pkgs/container/slamforge)

[中文版本 (Chinese Version)](README_ch.md)

**SLAMForge** is a C++20 **monocular visual SLAM and dense reconstruction** system. It estimates
6-DoF camera motion with geometric SLAM, then fuses locally inferred depth into a colored surface
map. Its tracking and mapping architecture is modeled after **ORB-SLAM3**.

---

## Quick Start

### Desktop beta — no development environment required

Download `SLAMForge Desktop 3.2.0-beta.1` from the
[GitHub Releases page](https://github.com/JackXing875/SLAMForge/releases/tag/v3.2.0-beta.1):

- **Windows x64:** extract the ZIP, then run `SLAMForge Desktop.exe`.
- **Linux x86_64:** make the AppImage executable, then launch it.

Drop a video into the window, select a YAML file calibrated for that camera, choose a results
directory, and start mapping. Processing is local; the application displays the completed dense
colored surface and trajectory and exports `map.ply`, `sparse_map.ply`, `trajectory.txt`, and
`run.log`.

> Monocular SLAM has an unknown absolute scale and requires accurate camera intrinsics. The desktop
> application does not infer calibration from arbitrary videos.

### Developer and container usage

```bash
# Build with Docker (zero host dependencies)
docker build -t slamforge -f docker/Dockerfile .
docker run --rm -v /path/to/images:/images -v $PWD/output:/output \
    slamforge run --config /opt/slamforge/config/kitti.yaml --input /images --output /output/traj.txt

# Or build natively (Ubuntu 22.04)
sudo apt-get install -y libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/apps/slamforge_cli run --config config/kitti.yaml --input /path/to/images

# Evaluate results
python3 tools/evaluate_ate.py output/traj.txt groundtruth.txt --plot
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Input: Monocular Video                  │
└──────────────────────┬──────────────────────────────────┘
                       │
         ┌─────────────▼─────────────┐
         │   TRACKING (real-time)    │
         │   • ORB feature extraction│
         │   • Frame-to-frame motion │
         │   • Local map tracking    │
         │   • Keyframe decision     │
         └─────────────┬─────────────┘
                       │ new KeyFrame
         ┌─────────────▼─────────────┐
         │   LOCAL MAPPING (async)   │
         │   • Map point triangulation│
         │   • Local BA (Ceres)      │
         │   • Point/KF culling      │
         └─────────────┬─────────────┘
                       │
         ┌─────────────▼─────────────┐
         │   LOOP CLOSING (async)    │
         │   • BoW loop detection    │
         │   • Sim(3) verification   │
         │   • Pose graph opt (g2o)  │
         │   • Global BA             │
         └─────────────┬─────────────┘
                       │ final poses + sparse anchors
         ┌─────────────▼─────────────┐
         │ DENSE RECONSTRUCTION      │
         │ • Local learned depth     │
         │ • Sparse scale calibration│
         │ • Multi-view consistency │
         │ • Colored voxel fusion   │
         └───────────────────────────┘
```

## Features

| Category      | Status | Description                                                       |
| ------------- | ------ | ----------------------------------------------------------------- |
| Tracking      | ✅      | Two-view init, motion model, local map tracking, relocalization   |
| Local Mapping | ✅      | Triangulation, local BA (Ceres), point/KF culling                 |
| Dense Mapping | 🧪      | Offline learned depth, sparse calibration, multi-view voxel fusion |
| Loop Closing  | ✅      | FBOW vocabulary, Sim(3) verification, pose graph (g2o), global BA |
| ORB Features  | ✅      | Multi-scale pyramid, quadtree distribution, adaptive threshold    |
| Configuration | ✅      | YAML-based with schema validation (camera, algorithm parameters)  |
| Desktop Beta  | 🧪      | Windows/Linux video workflow, dense result viewer and export       |
| CLI           | ✅      | `run`, `eval`, `benchmark` subcommands                            |
| ROS2 Node     | ✅      | Real-time SLAM with PoseStamped, PointCloud2, TF                  |
| Python API    | ✅      | pybind11 bindings with numpy interop                              |
| Evaluation    | ✅      | ATE, RPE, trajectory plotting, batch benchmarking                 |
| Docker        | ✅      | One-command build + run                                           |
| Documentation | ✅      | Doxygen API docs, architecture, quick start, tuning guide         |
| Unit Tests    | ✅      | 20 core/CLI tests plus a desktop result-viewer smoke test          |
| CI/CD         | ✅      | GitHub Actions: build, test, lint, docs, Docker                   |
| Benchmarks    | ✅      | Google Benchmark: ORB, PnP, triangulation                         |

## Reference Regression

The release gate includes two complete runs of a calibrated, rectified 4,757-frame TUM MonoVO
sequence. Both runs produced byte-identical trajectories and sparse maps.

| Poses | Lost after init | Keyframes | Rigid loops | Dense points | Partial-GT ATE RMSE |
| ----- | --------------- | --------- | ----------- | ------------ | ------------------- |
| 4,556 | 0               | 598       | 2           | 886,813      | 1.554 m             |

ATE uses global Sim(3) alignment over the 954 poses that have finite ground truth. This is a
single-sequence regression result, not a broad accuracy benchmark or a survey-grade claim.

## Technology Stack

| Layer          | Technology                   | Purpose                                   |
| -------------- | ---------------------------- | ----------------------------------------- |
| Language       | C++20                        | Core engine                               |
| Build          | CMake 3.20+                  | Build system, FetchContent deps           |
| Linear Algebra | Eigen 3.3+ + Sophus          | Matrix ops + SE(3) Lie groups             |
| Vision         | OpenCV 4.x                   | ORB, PnP, essential matrix, image I/O     |
| Dense Depth    | Depth Anything V2 Small + ONNX Runtime | Local monocular surface inference |
| Local Opt      | Ceres Solver                 | Local bundle adjustment                   |
| Pose Graph     | g2o                          | Loop closure pose graph optimization      |
| Vocabulary     | FBOW                         | DBoW2-compatible visual place recognition |
| Logging        | spdlog                       | Structured async logging                  |
| Config         | yaml-cpp                     | YAML runtime configuration                |
| CLI            | CLI11                        | Subcommand-based argument parsing         |
| Bindings       | pybind11                     | C++ → Python bridge                       |
| Testing        | GoogleTest, Google Benchmark | Unit tests + micro-benchmarks             |
| Container      | Docker                       | Reproducible build environment            |
| CI/CD          | GitHub Actions               | Automated build, test, lint, deploy       |

## Project Structure

```
SLAMForge/
├── include/slamforge/         # Public headers
│   ├── core/               # Types, Camera, Map, Frame, Config
│   ├── tracking/           # Tracker, Initializer, FeatureMatcher
│   ├── mapping/            # LocalMapper, DenseMapper
│   ├── loop_closing/       # Vocabulary, Detector, Verifier, PoseGraph, GlobalBA
│   ├── geometry/           # SE3, Sim3, Epipolar, PnP, Triangulation
│   ├── features/           # ORB extractor
│   └── optimization/       # Bundle Adjustment
├── src/                    # Implementation files
├── apps/                   # CLI and ROS2 applications
├── tests/
│   ├── unit/               # GoogleTest core and CLI regression tests
│   └── bench/              # Google Benchmark micro-benchmarks
├── tools/                  # Python evaluation scripts
├── pybind/                 # Python bindings
├── config/                 # YAML configs (KITTI, EuRoC, TUM)
├── docker/                 # Dockerfile + compose + devcontainer
└── docs/                   # Architecture, quick start, tuning guide
```

## CLI Usage

```bash
# Run SLAM on a directory of images
slamforge_cli run --config config/kitti.yaml --input /data/images --output traj.txt

# Preserve source timestamps for TUM/EuRoC-style image sequences
slamforge_cli run --config config/euroc.yaml --input /data/images \
    --timestamps /data/timestamps.txt --output traj.txt

# Run SLAM on a video file and request a dense colored map (desktop packages auto-find the model)
slamforge_cli run --config config/kitti.yaml --input /data/video.mp4 --fps 30 \
    --map-output sparse_map.ply --dense-output map.ply

# Evaluate trajectory against ground truth
slamforge_cli eval --estimated traj.txt --groundtruth gt.txt --format kitti

# Batch benchmark across dataset sequences
slamforge_cli benchmark --dataset-dir /data/kitti --config config/kitti.yaml
```

## Python API

```python
import numpy as np
import slamforge

# Load configuration
cfg = slamforge.load_config("config/kitti.yaml")

# Create camera and tracker
camera = slamforge.Camera(cfg.camera)
tracker = slamforge.Tracker(camera, cfg.tracking, cfg.orb)

# Track frames
for frame in frames:
    pose = tracker.track(frame, timestamp)
    if pose is not None:
        print(f"Position: {pose.position}")

# Access map
map_ = tracker.get_map()
print(f"Keyframes: {map_.keyframe_count}, Map points: {map_.map_point_count}")
```

## ROS2 Node

```bash
# Source your ROS2 installation, then build the node with ROS2 support
source /opt/ros/$ROS_DISTRO/setup.bash
cmake -B build -DSLAMFORGE_BUILD_ROS2=ON
cmake --build build

# Run the CMake-built node
./build/apps/slamforge_ros_node --ros-args -p config_path:=config/euroc.yaml
```

**Published topics:**
- `~/pose` — `geometry_msgs/PoseStamped`
- `~/map_cloud` — `sensor_msgs/PointCloud2`
- `~/keyframes` — `visualization_msgs/Marker`
- TF: `odom` → `camera_link`

## Building from Source

```bash
# Prerequisites
sudo apt-get install -y build-essential cmake libopencv-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev

# Optional: Ceres for bundle adjustment
sudo apt-get install -y libceres-dev libgoogle-glog-dev libgflags-dev

# Optional: Sophus for Lie algebra
git clone https://github.com/strasdat/Sophus.git && cd Sophus
cmake -B build && cmake --build build && sudo cmake --install build

# Build SLAMForge
cmake -B build -DCMAKE_BUILD_TYPE=Release \
    -DSLAMFORGE_BUILD_TESTS=ON \
    -DSLAMFORGE_ENABLE_CERES=ON
cmake --build build -j$(nproc)

# Run tests
cd build && ctest --output-on-failure
```

## Documentation

- [Quick Start Guide](docs/quick_start.md) — Get running in 5 minutes
- [Architecture Overview](docs/architecture.md) — System design and data flow
- [Tuning Guide](docs/tuning_guide.md) — Parameter optimization for your scene
- [Desktop Preview](docs/desktop.md) — Build and try the drag-and-drop Qt application
- [API Documentation](https://JackXing875.github.io/SLAMForge/) — Doxygen-generated class reference

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup, code style, and PR process.

## License

This project is licensed under the GNU General Public License v3.0 only. See [LICENSE](LICENSE) for details.

---

*Built with C++20 • Eigen • Sophus • OpenCV • Ceres • g2o • FBOW*
