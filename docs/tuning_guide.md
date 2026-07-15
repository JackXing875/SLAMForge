# SLAMForge Performance Tuning Guide

How to adjust SLAMForge parameters for your specific camera, scene, and performance requirements.

## Core Trade-offs

| Goal | Trade-off |
|------|-----------|
| Higher accuracy | More features, more keyframes → slower |
| Faster tracking | Fewer features, fewer pyramid levels → less robust |
| Better relocalization | More keyframes, BoW vocabulary → more memory |
| Lower memory | Aggressive culling → fewer map points |

## ORB Feature Extraction

```yaml
orb:
  num_features: 1200    # Target features per frame
  scale_factor: 1.2     # Pyramid scale factor
  num_levels: 8         # Pyramid levels
  ini_threshold: 20     # FAST initial threshold
  min_threshold: 7      # FAST minimum threshold
```

**Guidelines:**
- **`num_features`**: 1000–2000 is typical. Higher = more robust but slower. KITTI: 1200. EuRoC: 1000. Indoor/rapid motion: 1500+.
- **`scale_factor`**: 1.2 is the standard ORB-SLAM3 value. Larger = fewer levels but faster.
- **`num_levels`**: 8 gives ~4.3x scale coverage at 1.2. Reduce to 6 if memory-constrained.
- **`ini_threshold` / `min_threshold`**: Lower = more features in low-texture areas. Raise if too many spurious features.

## Tracking

```yaml
tracking:
  min_features_for_tracking: 50   # Lose tracking below this
  max_frames_between_kf: 40       # Force keyframe after N frames
  min_frames_between_kf: 5        # Don't create KFs too close
  max_reprojection_error: 4.0     # Max reprojection error (pixels)
  min_parallax_deg: 1.0           # Min parallax for triangulation
```

**Guidelines:**
- **`min_features_for_tracking`**: 30–100. Lower = keeps tracking longer but may drift. Higher = more conservative.
- **`max_frames_between_kf`**: 20–60. Too low = too many KFs, slow mapping. Too high = too few KFs, tracking may fail.
- **`min_frames_between_kf`**: 3–10. Prevents redundant keyframes.
- **`max_reprojection_error`**: 3.0–5.5 pixels. Tighter = fewer but better matches. Looser = more matches but more outliers.
- **`min_parallax_deg`**: 0.5–2.0. Smaller = triangulates with less viewpoint change (less accurate). Larger = waits for more parallax.

## Local Mapping

```yaml
mapping:
  sliding_window_size: 20      # Keyframes in local BA window
  min_observations: 3          # Min observations to keep a map point
  max_reprojection_error: 4.0  # Max reprojection for map points
```

**Guidelines:**
- **`sliding_window_size`**: 10–30. Larger = better optimization but slower. 20 is a good default.
- **`min_observations`**: 2–5. Higher = only very well-observed points survive. Lower = more points but more outliers.
- **`max_reprojection_error`**: Should match or be slightly looser than tracking threshold.

## Loop Closing

```yaml
loop_closing:
  enabled: true
  min_similarity_score: 0.3    # BoW similarity threshold
  min_consecutive_loops: 3     # Consecutive detections to accept
  pose_graph_iterations: 20    # g2o iterations
  global_ba_iterations: 20     # Global BA iterations
  enable_global_ba: true       # Run full BA after loop closure
```

**Guidelines:**
- **`min_similarity_score`**: 0.2–0.4. Lower = more loop candidates (more false positives). Higher = only strong matches.
- **`min_consecutive_loops`**: 2–5. Higher = fewer false positives but may miss real loops.
- **`pose_graph_iterations`**: 10–50. Usually converges within 20 iterations.
- **`enable_global_ba`**: Disable for speed, enable for maximum accuracy after loop closure.

## Scene-Specific Recommendations

### KITTI (outdoor, car, rectified)
```yaml
orb.num_features: 1200
tracking.min_features_for_tracking: 50
mapping.sliding_window_size: 20
```

### EuRoC (indoor, drone, distorted)
```yaml
orb.num_features: 1000
tracking.min_features_for_tracking: 40
tracking.min_parallax_deg: 0.5       # drone moves fast
mapping.sliding_window_size: 15
```

### TUM RGBD (indoor, handheld, rolling shutter)
```yaml
orb.num_features: 1000
tracking.min_features_for_tracking: 60  # motion blur
tracking.min_parallax_deg: 0.8
mapping.min_observations: 2             # more permissive
```

### Custom camera (unknown calibration quality)
```yaml
# Start conservative, then relax
orb.ini_threshold: 15
tracking.max_reprojection_error: 5.5
mapping.min_observations: 2
```

## Performance Profiling

Build with benchmarks:
```bash
cmake -B build -DSLAMFORGE_BUILD_BENCHMARKS=ON
cmake --build build
./build/tests/bench/bench_orb
```

The benchmark suite measures:
- ORB extraction throughput (features/second)
- PnP solve time (microseconds)
- Triangulation throughput (points/second)
- Bundle adjustment scaling (time vs. keyframes)
