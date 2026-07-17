SLAMForge Desktop 3.2.0-beta.1
================================

QUICK START

1. Extract the entire ZIP archive. Do not run the application from inside the ZIP.
2. Double-click "SLAMForge Desktop.exe".
3. Drop or select a monocular video.
4. Select a SLAMForge YAML file calibrated for that camera and video resolution.
5. Choose a writable results directory and click "Start mapping".
6. Inspect the dense colored surface map and trajectory in the 3D Result tab.

The results directory contains:

- trajectory.txt  Camera trajectory (format selected by the YAML file)
- map.ply         Fused dense colored surface cloud for CloudCompare, MeshLab, and similar tools
- sparse_map.ply  Geometric SLAM landmarks used to anchor the dense reconstruction
- run.log         Processing and diagnostic log

IMPORTANT LIMITATIONS

- Monocular SLAM has no absolute metric scale. The displayed map uses relative scale.
- Dense reconstruction is an offline second stage and takes additional processing time.
- Accurate camera intrinsics are required. Do not use a dataset preset for an unrelated camera.
- Low texture, motion blur, pure rotation, repeated patterns, and insufficient parallax may cause
  initialization or tracking failure.
- This is an unsigned public beta. Windows SmartScreen may show an unrecognized publisher warning.
- Built-in geometric loop closing is enabled; optional g2o/FBOW acceleration is not included.

PRIVACY AND LICENSE

Processing is local. SLAMForge does not upload the selected video or map.
SLAMForge is licensed under GPL-3.0-only. See LICENSE.txt.

Project: https://github.com/JackXing875/SLAMForge
