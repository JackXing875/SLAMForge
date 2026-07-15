# SLAMForge Python bindings

This package builds SLAMForge's native `pybind11` extension and exposes it as
`import slamforge`.

Building from source requires CMake, a C++20 compiler, OpenCV development
files, and Eigen3 development files.  Optional native SLAMForge features are
disabled for wheel builds so that the binding build does not fetch or bundle
large solver dependencies.
