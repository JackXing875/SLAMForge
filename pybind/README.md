# LiteVO Python bindings

This package builds LiteVO's native `pybind11` extension and exposes it as
`import litevo`.

Building from source requires CMake, a C++20 compiler, OpenCV development
files, and Eigen3 development files.  Optional native LiteVO features are
disabled for wheel builds so that the binding build does not fetch or bundle
large solver dependencies.
