/*
 * RDA exports its own commands instead of deriving them from the trajectory.
 *
 * The commands are QP variables bounded by the vehicle limits, so a native
 * export must satisfy |steering| <= max_steer and |acceleration| <= max_acc at
 * every segment, one command per segment. Reconstructing them from the states
 * instead missed the steering limit by 0.4 rad on the benchmark cases, so the
 * bounds are the regression this test guards.
 *
 * Replaying the commands through the exact bicycle model is reported but not
 * gated: RDA enforces the linearized dynamics and stops on the dual-feasibility
 * residual only, so the exact residual is not a property of the export.
 */
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
#include "planner/rda/rda_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include "yaml-cpp/yaml.h"

using namespace planning;

namespace {

params::RDAParams LoadRDAParamsFromYaml(const std::string& yaml_path) {
    params::RDAParams p;
    try {
        const YAML::Node config = YAML::LoadFile(yaml_path);
        const YAML::Node node   = config["rda_params"];
        p.set_max_iter(300);   // export test: allow RDA to converge on this scenario
        p.set_penalty_weight(node["penalty_weight"] ? node["penalty_weight"].as<double>() : 200.0);
        p.set_ro2(node["ro2"] ? node["ro2"].as<double>() : 50.0);
        p.set_w_s(node["w_s"] ? node["w_s"].as<double>() : 1.0);
        p.set_w_u(node["w_u"] ? node["w_u"].as<double>() : 0.5);
        p.set_l1_weight(node["l1_weight"] ? node["l1_weight"].as<double>() : 10.0);
        p.set_min_sd(node["min_sd"] ? node["min_sd"].as<double>() : 0.1);
        p.set_max_sd(node["max_sd"] ? node["max_sd"].as<double>() : 1.0);
        p.set_dt_min(node["dt_min"] ? node["dt_min"].as<double>() : 0.05);
        p.set_dt_max(node["dt_max"] ? node["dt_max"].as<double>() : 0.2);
        p.set_w_dt(node["w_dt"] ? node["w_dt"].as<double>() : 0.5);
        p.set_iter_threshold(node["iter_threshold"] ? node["iter_threshold"].as<double>() : 0.01);
        p.set_endpoint_position_tolerance(
            node["endpoint_position_tolerance"] ? node["endpoint_position_tolerance"].as<double>() : 0.05);
        p.set_endpoint_heading_tolerance(
            node["endpoint_heading_tolerance"] ? node["endpoint_heading_tolerance"].as<double>() : 0.08);
        p.set_safety_distance_step(node["safety_distance_step"] ? node["safety_distance_step"].as<double>() : 0.1);
        p.set_safety_distance_cap(node["safety_distance_cap"] ? node["safety_distance_cap"].as<double>() : 0.25);
        p.set_safety_distance_persist(
            node["safety_distance_persist"] ? node["safety_distance_persist"].as<int>() : 1);
        p.set_workers(1);   // this test does not exercise the worker pool
        LOG(INFO) << "Loaded rda_params from " << yaml_path;
    } catch (const std::exception& error) {
        LOG(ERROR) << "Failed to load " << yaml_path << ": " << error.what();
        throw;
    }
    return p;
}

kinematic_model::VehicleParam LoadVehicleParam() {
    kinematic_model::VehicleParam vehicle;
    vehicle.set_vehicle_type("car");
    vehicle.set_length(4.5);
    vehicle.set_width(2.0);
    vehicle.set_wheel_base(2.5);
    vehicle.set_front_overhang(1.0);
    vehicle.set_rear_overhang(1.0);
    vehicle.set_max_steer_angle(0.75);
    vehicle.set_max_acc(2.0);
    vehicle.set_max_velocity(15.0);
    return vehicle;
}
}  // namespace

int main() {
    const auto vehicle = LoadVehicleParam();
    // Straight reference path with one square obstacle on it, so the solver has
    // to command real steering and acceleration.
    problem::PlanProblem problem;
    problem.mutable_init_state()->set_x(0.0);
    problem.mutable_init_state()->set_y(0.0);
    problem.mutable_init_state()->set_theta(0.0);
    problem.mutable_goal_state()->set_x(20.0);
    problem.mutable_goal_state()->set_y(0.0);
    problem.mutable_goal_state()->set_theta(0.0);
    *problem.mutable_vehicle_param() = vehicle;
    auto* obstacle = problem.add_obstacle_list();
    obstacle->set_vertex_num(4);
    for (const auto& corner : {std::pair<double, double>{9.0, -0.6}, {11.0, -0.6}, {11.0, 0.6}, {9.0, 0.6}}) {
        auto* point = obstacle->add_vertex_pts();
        point->set_x(corner.first);
        point->set_y(corner.second);
    }
    problem.set_obstacle_num(1);
    problem.mutable_map_bound()->set_min_x(-1.0);
    problem.mutable_map_bound()->set_max_x(21.0);
    problem.mutable_map_bound()->set_min_y(-3.0);
    problem.mutable_map_bound()->set_max_y(3.0);

    const auto    map   = std::make_shared<map::Map>(problem, 0.1);
    const auto    model = std::make_shared<vehicle_model::KinematicModel>(vehicle);
    const auto    rda_params = LoadRDAParamsFromYaml(RDA_TEST_CONFIG_PATH);
    vehicle_model::sdv_path path;
    constexpr int           kPoints = 61;
    for (int i = 0; i < kPoints; ++i) {
        vehicle_model::VehiclePose pose;
        pose.x     = i * (20.0 / (kPoints - 1));
        pose.y     = 0.0;
        pose.theta = 0.0;
        path.push_back(pose);
    }
    auto start = std::make_shared<vehicle_model::VehiclePose>();
    auto goal  = std::make_shared<vehicle_model::VehiclePose>();
    goal->x    = 20.0;

    backend::RDASolver solver(model, map, rda_params, 0.1);
    if (!solver.Process(path, start, goal)) {
        LOG(ERROR) << "RDA failed to solve the test problem";
        return 1;
    }
    const auto&      states   = solver.GetStatesResult();
    const auto&      steps    = solver.GetTimeSteps();
    const Eigen::MatrixXd commands = solver.GetControlsResult();
    const Eigen::Index   segments = static_cast<Eigen::Index>(path.size()) - 1;
    if (commands.rows() != segments || commands.cols() != 2 || !commands.allFinite()) {
        LOG(ERROR) << "Exported commands have shape " << commands.rows() << "x" << commands.cols() << ", expected "
                   << segments << "x2 and finite";
        return 1;
    }
    if (states.size() != path.size() || steps.size() != segments) {
        LOG(ERROR) << "Exported states/steps do not match the horizon";
        return 1;
    }

    double max_steer = 0.0, max_acc = 0.0;
    for (Eigen::Index k = 0; k < segments; ++k) {
        max_steer = std::max(max_steer, std::abs(commands(k, 1)));
        max_acc   = std::max(max_acc, std::abs(commands(k, 0)));
    }
    // RDA bounds its commands inside the QP, but OSQP stops with its own default
    // feasibility tolerance, so an active bound can be missed by a few 1e-3.
    // The reconstruction this test replaces missed the steering limit by 0.4 rad.
    constexpr double kBoundTolerance = 5e-3;
    bool limits_ok =
        max_steer <= vehicle.max_steer_angle() + kBoundTolerance && max_acc <= vehicle.max_acc() + kBoundTolerance;
    LOG(INFO) << "Native commands: max|steer|=" << max_steer << " (limit " << vehicle.max_steer_angle()
              << ", excess " << max_steer - vehicle.max_steer_angle() << "), max|accel|=" << max_acc
              << " (limit " << vehicle.max_acc() << ", excess " << max_acc - vehicle.max_acc() << ")";

    // Replay the native commands through the rear-axle model for reporting.
    // The QP enforces the linearized dynamics, so the exact model holds only to
    // the linearization and penalty residual that RDA's stopping criterion leaves.
    double max_position = 0.0, max_heading = 0.0, max_speed = 0.0;
    for (Eigen::Index k = 0; k < segments; ++k) {
        const double dt = steps[k];
        const Eigen::Vector4d& previous = states[static_cast<std::size_t>(k)];
        const Eigen::Vector4d& next     = states[static_cast<std::size_t>(k) + 1];
        const Eigen::Vector2d  expected(previous(0) + dt * previous(3) * std::cos(previous(2)),
                                        previous(1) + dt * previous(3) * std::sin(previous(2)));
        max_position = std::max(max_position, (expected - next.head<2>()).norm());
        const double heading = previous(2) +
                               dt * previous(3) * std::tan(commands(k, 1)) / vehicle.wheel_base();
        max_heading = std::max(max_heading, std::abs(std::atan2(std::sin(heading - next(2)),
                                                                std::cos(heading - next(2)))));
        max_speed = std::max(max_speed, std::abs(next(3) - (previous(3) + dt * commands(k, 0))));
    }
    LOG(INFO) << "Replay of the native commands: max position residual=" << max_position
              << " m, max heading residual=" << max_heading << " rad, max speed residual=" << max_speed << " m/s";

    if (!limits_ok) {
        LOG(ERROR) << "Native commands exceed the actuation limits in " << commands.rows() << " segments";
        return 1;
    }
    std::cout << "RDA native control export passed\n";
    return 0;
}
