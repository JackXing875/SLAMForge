# SLAMForge Desktop 3.1 Beta

SLAMForge Desktop runs monocular SLAM without a terminal or development environment. Select or drop
a video, choose a calibrated camera YAML file and results directory, then start or cancel analysis
while viewing progress and logs. A completed run is displayed as an interactive sparse point cloud
and camera trajectory.

Processing is delegated to the packaged `slamforge_cli` worker for crash isolation. Videos and maps
remain on the local machine.

## Download

The `v3.1.0-beta.1` GitHub Release provides:

- a Windows x64 portable ZIP;
- a Linux x86_64 AppImage.

Windows users must extract the complete archive before running `SLAMForge Desktop.exe`. Linux users
can launch the AppImage after making it executable:

```bash
chmod +x SLAMForge-Desktop-3.1.0-beta.1-Linux-x86_64.AppImage
./SLAMForge-Desktop-3.1.0-beta.1-Linux-x86_64.AppImage
```

## Build from source

The downloadable packages do not require a compiler or Qt SDK. To build the desktop application on
Ubuntu 24.04:

```bash
sudo apt-get install -y build-essential cmake qt6-base-dev \
    libopencv-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev
```

Qt 6.2 or newer is required. The desktop option is disabled by default, so library and CLI builds do
not acquire a Qt dependency.

```bash
cmake -B build-desktop \
    -DCMAKE_BUILD_TYPE=Release \
    -DSLAMFORGE_BUILD_DESKTOP=ON
cmake --build build-desktop --target slamforge_desktop --parallel
./build-desktop/apps/slamforge_desktop
```

The desktop target currently requires `SLAMFORGE_BUILD_APPS=ON` because `slamforge_cli` is the
analysis worker. The application locates the CLI beside its own executable, which also matches the
installed layout.

## Current output

The selected results directory receives:

- `trajectory.txt` in the format selected by the YAML configuration;
- `map.ply`, an ASCII sparse point cloud usable in external 3D tools;
- `run.log` containing the engine output and progress messages.

The built-in 3D result tab supports drag-to-rotate, wheel zoom, and double-click view reset. The
input video is not copied to the results directory or uploaded.

## Calibration and monocular limitations

Do not select a dataset preset simply because its resolution resembles the video. Reliable SLAM
requires camera intrinsics and distortion parameters calibrated for the source camera. The preview
therefore does not silently choose a default camera configuration.

Monocular SLAM produces a sparse map with an unknown absolute scale. The viewer and exports are
explicitly labeled as relative scale. Calibration assistance and live per-frame 3D updates remain
future milestones.

This first package enables Ceres local bundle adjustment but omits the optional g2o/FBOW
loop-closing backend. The limitation keeps the Windows and Linux beta packages reproducible while
cross-platform backend packaging is completed.
