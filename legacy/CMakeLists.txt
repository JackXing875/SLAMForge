cmake_minimum_required(VERSION 3.14)
project(DeepVO_CPP VERSION 1.0.0 LANGUAGES CXX)

# Enforce C++17 standard for modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find core dependencies
# OpenMP accelerates OpenCV and Eigen matrix operations
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add header file search path
include_directories(include)

# Create a library containing all core implementations
# We will add tracker.cpp and epipolar.cpp here later
add_library(deepvo_core SHARED
    src/geometry/epipolar.cpp
    src/tracker.cpp
    src/visualizer.cpp
)

# Link dependencies to the core library
target_link_libraries(deepvo_core PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
)

# Build the main executable
add_executable(deepvo_app app/main.cpp)

# Link the core library to the executable
target_link_libraries(deepvo_app PRIVATE deepvo_core)