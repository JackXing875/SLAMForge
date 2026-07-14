"""Conan recipe for LiteVO — Industrial-Grade Monocular Visual SLAM."""

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy


class LiteVOConan(ConanFile):
    name = "litevo"
    version = "2.0.0"
    description = "Industrial-grade monocular visual SLAM system"
    license = "MIT"
    url = "https://github.com/yourname/LiteVO"
    topics = ("slam", "visual-odometry", "computer-vision", "robotics")

    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_sophus": [True, False],
        "with_ceres": [True, False],
        "with_g2o": [True, False],
        "with_fbow": [True, False],
        "with_spdlog": [True, False],
        "with_yaml": [True, False],
        "build_tests": [True, False],
        "build_apps": [True, False],
    }
    default_options = {
        "with_sophus": True,
        "with_ceres": True,
        "with_g2o": True,
        "with_fbow": True,
        "with_spdlog": True,
        "with_yaml": True,
        "build_tests": False,
        "build_apps": True,
    }

    requires = "eigen/3.4.0", "opencv/4.8.0"

    def requirements(self):
        if self.options.with_sophus:
            self.requires("sophus/1.22.10")
        if self.options.with_ceres:
            self.requires("ceres-solver/2.2.0")
        if self.options.with_g2o:
            self.requires("g2o/20230223")
        if self.options.with_spdlog:
            self.requires("spdlog/1.13.0")
        if self.options.with_yaml:
            self.requires("yaml-cpp/0.8.0")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["LITEVO_ENABLE_SOPHUS"] = self.options.with_sophus
        tc.variables["LITEVO_ENABLE_CERES"] = self.options.with_ceres
        tc.variables["LITEVO_ENABLE_G2O"] = self.options.with_g2o
        tc.variables["LITEVO_ENABLE_FBOW"] = self.options.with_fbow
        tc.variables["LITEVO_ENABLE_SPDLOG"] = self.options.with_spdlog
        tc.variables["LITEVO_ENABLE_YAML"] = self.options.with_yaml
        tc.variables["LITEVO_BUILD_TESTS"] = self.options.build_tests
        tc.variables["LITEVO_BUILD_APPS"] = self.options.build_apps
        tc.variables["LITEVO_BUILD_PYBIND"] = False
        tc.variables["LITEVO_BUILD_ROS2"] = False
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()
        copy(self, "LICENSE", src=self.source_folder, dst=self.package_folder)

    def package_info(self):
        self.cpp_info.libs = ["litevo_core"]
        self.cpp_info.includedirs = ["include"]
