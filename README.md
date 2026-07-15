# LiteVO — Industrial-Grade Monocular Visual SLAM

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue?logo=c%2B%2B)](https://isocpp.org/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-latest-2496ED?logo=docker)](https://github.com/yourname/LiteVO/pkgs/container/litevo)

[中文版本 (Chinese Version)](README_ch.md)

**LiteVO** is an industrial-grade **monocular visual SLAM** system built from scratch in C++20. It estimates 6-DoF camera motion and builds a sparse 3D map from a single video stream in real time — with an architecture modeled after **ORB-SLAM3**.

---

## Quick Start

```bash
# Build with Docker (zero host dependencies)
docker build -t litevo -f docker/Dockerfile .
docker run --rm -v /path/to/images:/images -v $PWD/output:/output \
    litevo run --config /opt/litevo/config/kitti.yaml --input /images --output /output/traj.txt

# Or build natively (Ubuntu 22.04)
sudo apt-get install -y libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/apps/litevo_cli run --config config/kitti.yaml --input /path/to/images

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
         └───────────────────────────┘
```

## Features

| Category      | Status | Description                                                       |
| ------------- | ------ | ----------------------------------------------------------------- |
| Tracking      | ✅      | Two-view init, motion model, local map tracking, relocalization   |
| Local Mapping | ✅      | Triangulation, local BA (Ceres), point/KF culling                 |
| Loop Closing  | ✅      | FBOW vocabulary, Sim(3) verification, pose graph (g2o), global BA |
| ORB Features  | ✅      | Multi-scale pyramid, quadtree distribution, adaptive threshold    |
| Configuration | ✅      | YAML-based with schema validation (camera, algorithm parameters)  |
| CLI           | ✅      | `run`, `eval`, `benchmark` subcommands                            |
| ROS2 Node     | ✅      | Real-time SLAM with PoseStamped, PointCloud2, TF                  |
| Python API    | ✅      | pybind11 bindings with numpy interop                              |
| Evaluation    | ✅      | ATE, RPE, trajectory plotting, batch benchmarking                 |
| Docker        | ✅      | One-command build + run                                           |
| Documentation | ✅      | Doxygen API docs, architecture, quick start, tuning guide         |
| Unit Tests    | ✅      | 12 test suites with GoogleTest                                    |
| CI/CD         | ✅      | GitHub Actions: build, test, lint, docs, Docker                   |
| Benchmarks    | ✅      | Google Benchmark: ORB, PnP, triangulation                         |

## Performance Benchmarks

*Results on KITTI odometry benchmark (placeholder — actual results pending dataset run):*

| Sequence | ATE RMSE (m) | RPE trans (m) | RPE rot (°/m) | KFs | MPs |
| -------- | ------------ | ------------- | ------------- | --- | --- |
| KITTI 00 | —            | —             | —             | —   | —   |
| KITTI 01 | —            | —             | —             | —   | —   |
| KITTI 02 | —            | —             | —             | —   | —   |

## Technology Stack

| Layer          | Technology                   | Purpose                                   |
| -------------- | ---------------------------- | ----------------------------------------- |
| Language       | C++20                        | Core engine                               |
| Build          | CMake 3.20+                  | Build system, FetchContent deps           |
| Linear Algebra | Eigen 3.3+ + Sophus          | Matrix ops + SE(3) Lie groups             |
| Vision         | OpenCV 4.x                   | ORB, PnP, essential matrix, image I/O     |
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
LiteVO/
├── include/litevo/         # Public headers
│   ├── core/               # Types, Camera, Map, Frame, Config
│   ├── tracking/           # Tracker, Initializer, FeatureMatcher
│   ├── mapping/            # LocalMapper
│   ├── loop_closing/       # Vocabulary, Detector, Verifier, PoseGraph, GlobalBA
│   ├── geometry/           # SE3, Sim3, Epipolar, PnP, Triangulation
│   ├── features/           # ORB extractor
│   └── optimization/       # Bundle Adjustment
├── src/                    # Implementation files
├── apps/                   # CLI and ROS2 applications
├── tests/
│   ├── unit/               # GoogleTest unit tests (12 suites)
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
litevo_cli run --config config/kitti.yaml --input /data/images --output traj.txt

# Preserve source timestamps for TUM/EuRoC-style image sequences
litevo_cli run --config config/euroc.yaml --input /data/images \
    --timestamps /data/timestamps.txt --output traj.txt

# Run SLAM on a video file
litevo_cli run --config config/kitti.yaml --input /data/video.mp4 --fps 30

# Evaluate trajectory against ground truth
litevo_cli eval --estimated traj.txt --groundtruth gt.txt --format kitti

# Batch benchmark across dataset sequences
litevo_cli benchmark --dataset-dir /data/kitti --config config/kitti.yaml
```

## Python API

```python
import numpy as np
import litevo

# Load configuration
cfg = litevo.load_config("config/kitti.yaml")

# Create camera and tracker
camera = litevo.Camera(cfg.camera)
tracker = litevo.Tracker(camera, cfg.tracking, cfg.orb)

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
cmake -B build -DLITEVO_BUILD_ROS2=ON
cmake --build build

# Run the CMake-built node
./build/apps/litevo_ros_node --ros-args -p config_path:=config/euroc.yaml
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

# Build LiteVO
cmake -B build -DCMAKE_BUILD_TYPE=Release \
    -DLITEVO_BUILD_TESTS=ON \
    -DLITEVO_ENABLE_CERES=ON
cmake --build build -j$(nproc)

# Run tests
cd build && ctest --output-on-failure
```

## Documentation

- [Quick Start Guide](docs/quick_start.md) — Get running in 5 minutes
- [Architecture Overview](docs/architecture.md) — System design and data flow
- [Tuning Guide](docs/tuning_guide.md) — Parameter optimization for your scene
- [API Documentation](https://yourname.github.io/LiteVO/) — Doxygen-generated class reference

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup, code style, and PR process.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

*Built with C++20 • Eigen • Sophus • OpenCV • Ceres • g2o • FBOW*
