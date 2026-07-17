# SLAMForge Desktop 3.2 Beta

SLAMForge Desktop runs monocular SLAM without a terminal or development environment. Select or drop
a video, choose a calibrated camera YAML file and results directory, then start or cancel analysis
while viewing progress and logs. A completed run is displayed as an interactive dense colored
surface cloud and camera trajectory.

Processing is delegated to the packaged `slamforge_cli` worker for crash isolation. Videos and maps
remain on the local machine.

## Download

The `v3.2.0-beta.1` GitHub Release provides:

- a Windows x64 portable ZIP;
- a Linux x86_64 AppImage.

Windows users must extract the complete archive before running `SLAMForge Desktop.exe`. Linux users
can launch the AppImage after making it executable:

```bash
chmod +x SLAMForge-Desktop-3.2.0-beta.1-Linux-x86_64.AppImage
./SLAMForge-Desktop-3.2.0-beta.1-Linux-x86_64.AppImage
```

## Build from source

The downloadable packages do not require a compiler or Qt SDK. To build the desktop application on
Ubuntu 24.04:

```bash
sudo apt-get install -y build-essential cmake qt6-base-dev \
    libceres-dev libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev
```

Qt 6.2 or newer is required. The desktop option is disabled by default, so library and CLI builds do
not acquire a Qt dependency. Dense source builds also require an official ONNX Runtime C/C++ binary
package and the Apache-2.0 Depth Anything V2 Small ONNX model. Set their local paths explicitly:

```bash
cmake -B build-desktop \
    -DCMAKE_BUILD_TYPE=Release \
    -DSLAMFORGE_BUILD_DESKTOP=ON \
    -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64 \
    -DSLAMFORGE_DEPTH_MODEL_FILE=/path/to/depth_anything_v2_vits_dynamic.onnx
cmake --build build-desktop --target slamforge_desktop --parallel
./build-desktop/apps/slamforge_desktop
```

The desktop target currently requires `SLAMFORGE_BUILD_APPS=ON` because `slamforge_cli` is the
analysis worker. The application locates the CLI beside its own executable, which also matches the
installed layout.

## Current output

The selected results directory receives:

- `trajectory.txt` in the format selected by the YAML configuration;
- `map.ply`, a fused colored dense surface cloud usable in external 3D tools;
- `sparse_map.ply`, the geometric SLAM landmarks used as dense-depth anchors;
- `run.log` containing the engine output and progress messages.

The built-in 3D result tab supports drag-to-rotate, wheel zoom, and double-click view reset. The
input video is not copied to the results directory or uploaded.

## Calibration and monocular limitations

Do not select a dataset preset simply because its resolution resembles the video. Reliable SLAM
requires camera intrinsics and distortion parameters calibrated for the source camera. The preview
therefore does not silently choose a default camera configuration.

The packaged `tum_mono_sequence01_rectified.yaml` preset is intentionally limited to TUM Mono-VO
sequence_01 after conversion to the documented 640x480 pinhole projection. It is not a generic
640x480 calibration and must not be used with the raw fisheye video.

Monocular SLAM still has an unknown absolute scale. Depth Anything V2 Small predicts a continuous
relative depth field; SLAMForge robustly aligns that prediction to sparse landmarks in each selected
keyframe, rejects inconsistent points in adjacent views, and fuses the survivors into colored
voxels. This makes walls and large objects visible, but it does not turn monocular input into survey
grade geometry. The viewer and exports remain explicitly labeled as relative scale.

Dense inference is entirely local and adds an offline second phase after tracking. The model is
bundled in the downloadable package, so end users do not need Python, an internet connection, or a
separate model download.

This package enables Ceres local bundle adjustment and the deterministic built-in geometric loop
closing fallback. It omits the optional g2o/FBOW acceleration backends to keep the Windows and Linux
beta packages reproducible while cross-platform backend packaging is completed.
