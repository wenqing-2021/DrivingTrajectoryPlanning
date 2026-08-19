#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "planner/mppi/mppi_planner.h"

namespace py = pybind11;

PYBIND11_MODULE(mppi_pybind, module) {
    module.doc() = "C++ MPPI trajectory planner for CommonRoad urban scenarios";

    py::class_<planning::mppi::VehicleState>(module, "VehicleState")
        .def(py::init<>())
        .def_readwrite("x", &planning::mppi::VehicleState::x)
        .def_readwrite("y", &planning::mppi::VehicleState::y)
        .def_readwrite("yaw", &planning::mppi::VehicleState::yaw)
        .def_readwrite("velocity", &planning::mppi::VehicleState::velocity);

    py::class_<planning::mppi::Control>(module, "Control")
        .def(py::init<>())
        .def_readwrite("steering", &planning::mppi::Control::steering)
        .def_readwrite("acceleration", &planning::mppi::Control::acceleration);

    py::class_<planning::mppi::CircularObstacle>(module, "CircularObstacle")
        .def(py::init<>())
        .def_readwrite("x", &planning::mppi::CircularObstacle::x)
        .def_readwrite("y", &planning::mppi::CircularObstacle::y)
        .def_readwrite("radius", &planning::mppi::CircularObstacle::radius);

    py::class_<planning::mppi::MppiConfig>(module, "MppiConfig")
        .def(py::init<>())
        .def_readwrite("horizon", &planning::mppi::MppiConfig::horizon)
        .def_readwrite("num_samples", &planning::mppi::MppiConfig::num_samples)
        .def_readwrite("num_iterations", &planning::mppi::MppiConfig::num_iterations)
        .def_readwrite("time_step", &planning::mppi::MppiConfig::time_step)
        .def_readwrite("temperature", &planning::mppi::MppiConfig::temperature)
        .def_readwrite("wheelbase", &planning::mppi::MppiConfig::wheelbase)
        .def_readwrite("min_steering", &planning::mppi::MppiConfig::min_steering)
        .def_readwrite("max_steering", &planning::mppi::MppiConfig::max_steering)
        .def_readwrite("min_acceleration", &planning::mppi::MppiConfig::min_acceleration)
        .def_readwrite("max_acceleration", &planning::mppi::MppiConfig::max_acceleration)
        .def_readwrite("min_velocity", &planning::mppi::MppiConfig::min_velocity)
        .def_readwrite("max_velocity", &planning::mppi::MppiConfig::max_velocity)
        .def_readwrite("steering_noise", &planning::mppi::MppiConfig::steering_noise)
        .def_readwrite("acceleration_noise", &planning::mppi::MppiConfig::acceleration_noise)
        .def_readwrite("noise_smoothing", &planning::mppi::MppiConfig::noise_smoothing)
        .def_readwrite("position_weight", &planning::mppi::MppiConfig::position_weight)
        .def_readwrite("heading_weight", &planning::mppi::MppiConfig::heading_weight)
        .def_readwrite("velocity_weight", &planning::mppi::MppiConfig::velocity_weight)
        .def_readwrite("control_weight", &planning::mppi::MppiConfig::control_weight)
        .def_readwrite("control_rate_weight", &planning::mppi::MppiConfig::control_rate_weight)
        .def_readwrite("terminal_weight", &planning::mppi::MppiConfig::terminal_weight)
        .def_readwrite("obstacle_weight", &planning::mppi::MppiConfig::obstacle_weight)
        .def_readwrite("obstacle_softening", &planning::mppi::MppiConfig::obstacle_softening)
        .def_readwrite("collision_cost", &planning::mppi::MppiConfig::collision_cost)
        .def_readwrite("vehicle_radius", &planning::mppi::MppiConfig::vehicle_radius)
        .def_readwrite("road_left_limit", &planning::mppi::MppiConfig::road_left_limit)
        .def_readwrite("road_right_limit", &planning::mppi::MppiConfig::road_right_limit)
        .def_readwrite("road_boundary_weight", &planning::mppi::MppiConfig::road_boundary_weight)
        .def_readwrite("random_seed", &planning::mppi::MppiConfig::random_seed);

    py::class_<planning::mppi::PlanResult>(module, "PlanResult")
        .def_readonly("states", &planning::mppi::PlanResult::states)
        .def_readonly("controls", &planning::mppi::PlanResult::controls)
        .def_readonly("cost", &planning::mppi::PlanResult::cost);

    py::class_<planning::mppi::MppiPlanner>(module, "MppiPlanner")
        .def(py::init<planning::mppi::MppiConfig>())
        .def("plan",
             &planning::mppi::MppiPlanner::Plan,
             py::arg("initial_state"),
             py::arg("reference"),
             py::arg("obstacles"))
        .def("reset", &planning::mppi::MppiPlanner::Reset)
        .def_property_readonly(
            "config", &planning::mppi::MppiPlanner::config, py::return_value_policy::reference_internal);
}
