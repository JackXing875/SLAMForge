cmake_minimum_required(VERSION 3.20)

project(litevo_package_smoke LANGUAGES CXX)

find_package(litevo 2.0 CONFIG REQUIRED)

add_executable(litevo_package_smoke main.cpp)
target_link_libraries(litevo_package_smoke PRIVATE litevo::litevo_core)
