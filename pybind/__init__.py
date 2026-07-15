"""SLAMForge — Industrial-Grade Monocular Visual SLAM with Python Bindings.

Quickstart
----------
    import numpy as np
    from slamforge import *

    # Load configuration
    cfg = load_config("kitti.yaml")

    # Create camera from config dict
    cam = Camera(cfg["camera"]["fx"], cfg["camera"]["fy"],
                 cfg["camera"]["cx"], cfg["camera"]["cy"],
                 cfg["camera"]["width"], cfg["camera"]["height"])
    cam.set_distortion(cfg["camera"]["k1"], cfg["camera"]["k2"],
                       cfg["camera"]["p1"], cfg["camera"]["p2"],
                       cfg["camera"]["k3"])

    # Create tracker
    tracker = Tracker(cam, TrackingConfig(), OrbConfig())

    # Feed frames
    for img in frames:  # img: (H, W) uint8 numpy array
        pose = tracker.track(img, timestamp)
        if pose is not None:
            print("Tcw =\\n", pose)

    # Inspect map
    mp = tracker.get_map()
    for pt in mp.get_all_map_points():
        print(pt.position)
"""

from _slamforge import *
