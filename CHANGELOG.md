# Changelog

All notable changes to SLAMForge will be documented in this file.

## [3.1.0-beta.1] — 2026-07-15

### Added

- First public SLAMForge Desktop beta for Windows x64 and Linux x86_64.
- Drag-and-drop video workflow with camera configuration and local result directory selection.
- Interactive sparse-map and camera-trajectory result viewer with relative-scale disclosure.
- Thread-safe ASCII PLY sparse-map export through the C++ API and `--map-output` CLI option.
- Cancellable isolated SLAM worker process, progress reporting, persistent run logs, and result
  export.
- Cross-platform release automation for a Windows portable ZIP and Linux AppImage.

### Known limitations

- Camera intrinsics must be supplied through a calibrated SLAMForge YAML file.
- Monocular results have an unknown absolute scale.
- This beta renders completed results; live per-frame 3D updates remain planned.
- Release packages do not currently enable the optional g2o/FBOW loop-closing backend.

## [3.0.0] — 2026-07-15

### Breaking changes

- Renamed the project and all public interfaces to **SLAMForge**: C++ headers,
  namespace, CMake package, CLI, ROS2 node, Python package, and build options.
- Relicensed the project under **GPL-3.0-only**.

### Release readiness

- Added relocatable installation resources, package-consumer smoke coverage,
  and CI checks for formatting, static analysis, and Docker delivery.

## [2.0.0] — 2026-07-14

### Added
- Complete ORB-SLAM3-inspired architecture with three-thread design
- **Tracking**: Two-view initialization, constant-velocity motion model, local map tracking, relocalization
- **Local Mapping**: Map point triangulation, Ceres-based local bundle adjustment, keyframe/point culling
- **Loop Closing**: FBOW-based visual vocabulary, loop detection with geometric verification, Sim(3) correction, g2o pose graph optimization, global BA
- ORB feature extractor with quadtree distribution
- SE(3) and Sim(3) Lie algebra utilities
- Comprehensive geometry tools: Epipolar geometry (essential/fundamental/homography), PnP, triangulation
- YAML-based configuration system with schema
- CLI application with `run`, `eval`, and `benchmark` subcommands
- ROS2 node for real-time SLAM
- Python bindings via pybind11
- Evaluation tools: ATE, RPE, trajectory plotting, batch benchmarking
- Docker support with one-command build
- VS Code devcontainer configuration
- Comprehensive Doxygen API documentation
- GoogleTest unit tests (12 test suites)
- Google Benchmark micro-benchmarks (ORB, PnP, triangulation)
- GitHub Actions CI (build + test + lint + docs + Docker)
- CPack and Conan packaging
- English and Chinese documentation

### Changed
- Complete rewrite from the 1,327-line prototype to industrial-grade system
- C++20 standard with modern CMake practices
- Modular architecture with clear subsystem boundaries

### Fixed
- N/A (this is the first release of the new architecture)

## [1.0.0] — 2025-03-01

### Added
- Initial prototype: basic monocular VO with KLT tracking
- Essential matrix estimation (MAGSAC)
- PnP pose estimation
- Triangulation with quality filters
- CSV trajectory output
- Basic matplotlib 3D viewer
