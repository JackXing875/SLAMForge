# Changelog

All notable changes to SLAMForge will be documented in this file.

## [3.2.0-beta.1] — 2026-07-17

### Added

- Added offline dense reconstruction with the Apache-2.0 Depth Anything V2 Small model and ONNX
  Runtime, bundled in Windows and Linux desktop packages for fully local inference.
- Added robust per-keyframe depth scale/shift calibration from sparse SLAM landmarks, adjacent-view
  consistency filtering, deterministic voxel fusion, and colored ASCII PLY export.
- Added a dense colored desktop viewer and two-result workflow: `map.ply` for the fused surface and
  `sparse_map.ply` for geometric landmarks.
- Added dense reconstruction YAML tuning, CLI `--dense-output` / `--depth-model` options, and depth
  calibration regression coverage.

### Fixed

- Rejected weak extreme-scale loop candidates and changed verified PnP loop application to rigid
  endpoint correction, preventing sparse closures from expanding route interiors by 20–30x.
- Made equal-weight covisibility and local-BA residual ordering deterministic, preventing a
  long-video numerical branch that previously caused tracking loss and a discontinuous pose jump.

### Packaging

- Pinned and checksum-verifies ONNX Runtime 1.27.1 and the 99 MB Depth Anything V2 Small ONNX model
  in the release workflow; no Python installation or runtime download is required.

### Validation

- Two complete runs of the 4,757-frame rectified reference sequence produced byte-identical
  trajectories and sparse maps: 4,556 poses, no post-initialization lost frames, 598 keyframes,
  two rigid loop corrections, and an 886,813-point dense surface cloud.
- The partial 954-pose ground-truth interval measured 1.554 m Sim(3)-aligned ATE RMSE. This is a
  regression result for one calibrated sequence, not a general accuracy benchmark.

## [3.1.0-beta.2] — 2026-07-16

### Changed

- Reworked video tracking with robust PnP, optical-flow support, essential-matrix recovery, and
  safer pose synchronization after local optimization.
- Rebuilt landmark triangulation and local bundle adjustment to reject underconstrained or invalid
  residuals instead of allowing unstable solver steps to corrupt the map.
- Added deterministic batch finalization and multi-loop Sim(3) drift correction, including a
  geometric fallback that remains available without optional FBOW or g2o dependencies.
- Improved trajectory and sparse-map exports with stable ordering and outlier filtering.

### Fixed

- Prevented long videos from remaining permanently lost after temporary tracking degradation.
- Prevented non-finite Ceres steps and dense Cholesky failures caused by points crossing the camera
  plane or single-view landmarks entering bundle adjustment.
- Rejected false loop closures that could rotate the reconstructed map by approximately 180 degrees.

### Validation

- The 4,757-frame reference sequence now exports 4,554 poses, closes the complete route, and no
  longer emits the repeated linear-solver failures seen in beta.1.
- All build, unit, desktop, documentation, lint, and Docker CI workflows pass on the release source.

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
