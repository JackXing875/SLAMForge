# SLAMForge Quick Start Guide

Get SLAMForge running on your own images in 5 minutes.

## Prerequisites

- **Docker** (recommended) — zero host dependencies
- **Or**: Ubuntu 22.04+ with build tools

## Option 1: Docker (Recommended)

```bash
# 1. Clone and build
git clone https://github.com/JackXing875/SLAMForge.git && cd SLAMForge
docker build -t slamforge -f docker/Dockerfile .

# 2. Run on your images
docker run --rm -v /path/to/images:/images -v $PWD/output:/output \
    slamforge run --config /opt/slamforge/config/kitti.yaml \
    --input /images --output /output/traj.txt --verbose

# 3. Evaluate against ground truth
python3 tools/evaluate_ate.py output/traj.txt /path/to/groundtruth.txt --plot
```

## Option 2: Native Build (Ubuntu 22.04)

```bash
# 1. Install system dependencies
sudo apt-get install -y build-essential cmake libopencv-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libceres-dev

# 2. (Optional) Install Sophus
git clone https://github.com/strasdat/Sophus.git && cd Sophus
cmake -B build && cmake --build build && sudo cmake --install build

# 3. Build SLAMForge
cd SLAMForge
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 4. Run
./build/apps/slamforge_cli run --config config/kitti.yaml \
    --input /path/to/images --output traj.txt

# Supply timestamps for TUM/EuRoC-style image sequences when available
./build/apps/slamforge_cli run --config config/euroc.yaml \
    --input /path/to/images --timestamps /path/to/timestamps.txt --output traj.txt
```

## Prepare Your Data

### From a video file
```bash
# Extract frames at 10 FPS
mkdir -p images
ffmpeg -i video.mp4 -r 10 images/%06d.png
```

### From a KITTI sequence
Download any sequence from the [KITTI odometry benchmark](https://www.cit.tu-berlin.de/odom/). The image directory is `{sequence}/image_0/`.

### Required camera calibration
You need a YAML config with your camera intrinsics. Start from `config/kitti.yaml` and edit the values:

```yaml
camera:
  width: 1920
  height: 1080
  fx: 1000.0   # your focal length
  fy: 1000.0
  cx: 960.0    # principal point (width/2)
  cy: 540.0    # principal point (height/2)
```

## Evaluate Results

```bash
# Absolute Trajectory Error
python3 tools/evaluate_ate.py output/traj.txt groundtruth.txt --format tum --plot

# Relative Pose Error
python3 tools/evaluate_rpe.py output/traj.txt groundtruth.txt --delta 1 --format tum
```

## What Next?

- [Tuning Guide](tuning_guide.md) — adjust parameters for your camera/scene
- [Architecture](architecture.md) — understand the system design
- [Python API](../pybind/) — use SLAMForge from Python
