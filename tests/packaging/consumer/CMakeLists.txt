cmake_minimum_required(VERSION 3.20)

project(slamforge_package_smoke LANGUAGES CXX)

find_package(SLAMForge 3.0 CONFIG REQUIRED)

add_executable(slamforge_package_smoke main.cpp)
target_link_libraries(slamforge_package_smoke PRIVATE SLAMForge::slamforge_core)
