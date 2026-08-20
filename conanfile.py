"""Conan dependency manifest for DrivingTrajectoryPlanning."""

from conan import ConanFile


class Driving(ConanFile):
    """Native dependencies consumed by the top-level CMake project."""

    settings = "os", "arch", "compiler", "build_type"
    generators = "CMakeDeps", "CMakeToolchain", "VirtualBuildEnv"
    tool_requires = "protobuf/3.21.12"

    requires = (
        "eigen/3.4.0",
        "fmt/10.2.1",
        "glog/0.6.0",
        "gtest/1.15.0",
        "protobuf/3.21.12",
        "pybind11/2.13.6",
        "yaml-cpp/0.8.0",
    )

    default_options = {
        "glog/*:shared": False,
        "glog/*:with_gflags": False,
        "glog/*:with_unwind": False,
        "protobuf/*:shared": False,
        "yaml-cpp/*:shared": False,
    }
