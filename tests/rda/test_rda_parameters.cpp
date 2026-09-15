#include "planner/rda/rda_planner.h"
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

// A stationary trajectory isolates parameter handling from obstacle avoidance.
bool solve(const params::RDAParams& config, double goal_x = 0.0, double goal_heading = 0.0) {
    kinematic_model::VehicleParam vehicle;
    vehicle.set_length(4.5);
    vehicle.set_width(2.0);
    vehicle.set_wheel_base(2.5);
    vehicle.set_rear_overhang(1.0);
    vehicle.set_max_velocity(5.0);
    vehicle.set_max_acc(2.0);
    vehicle.set_max_steer_angle(0.5);
    problem::PlanProblem problem;
    *problem.mutable_vehicle_param() = vehicle;
    auto* bounds = problem.mutable_map_bound();
    bounds->set_min_x(-10); bounds->set_max_x(10);
    bounds->set_min_y(-10); bounds->set_max_y(10);
    auto model = std::make_shared<planning::vehicle_model::KinematicModel>(vehicle);
    auto map = std::make_shared<map::Map>(problem, 0.3);
    planning::backend::RDASolver solver(model, map, config, 0.1);
    auto start = std::make_shared<planning::vehicle_model::VehiclePose>();
    start->x = 0; start->y = 0; start->theta = 0;
    auto goal = std::make_shared<planning::vehicle_model::VehiclePose>(*start);
    goal->x = goal_x;
    goal->theta = goal_heading;
    planning::vehicle_model::sdv_path path(3, *start);
    return solver.Process(path, start, goal);
}

void require(bool condition, const char* message) {
    if (!condition) { throw std::runtime_error(message); }
}

int main() {
    try {
        params::RDAParams config;
        config.set_max_iter(2);
        require(solve(config), "Missing optional fields must retain working defaults");

        auto legacy = config;
        legacy.set_convergence_tolerance(0.4);
        require(solve(legacy, 0.3), "Legacy endpoint tolerance must remain supported");
        legacy.set_endpoint_position_tolerance(0.0);
        require(!solve(legacy, 0.3), "Explicit position tolerance must override the legacy field");
        require(solve(legacy, 0.0, 0.3), "Missing heading tolerance must retain legacy fallback");
        legacy.set_endpoint_heading_tolerance(0.0);
        require(!solve(legacy, 0.0, 0.3), "Explicit heading tolerance must override the legacy field");

        config.set_min_sd(0.0);
        config.set_max_sd(0.0);
        config.set_endpoint_position_tolerance(0.0);
        config.set_endpoint_heading_tolerance(0.0);
        config.set_safety_distance_step(0.0);
        config.set_safety_distance_cap(0.0);
        // Presence must survive the same protobuf serialization used by bindings.
        params::RDAParams decoded;
        require(decoded.ParseFromString(config.SerializeAsString()), "Protobuf round trip failed");
        require(decoded.has_min_sd() && decoded.has_max_sd() && decoded.has_safety_distance_step(),
                "Explicit zeros must retain presence");
        require(solve(decoded), "Zero distance bounds and exact stationary endpoints must be valid");

        config.set_min_sd(0.4);
        config.set_max_sd(0.6);
        config.set_safety_distance_cap(0.25);
        require(solve(config), "A cap below min_sd must be accepted");
        config.set_safety_distance_cap(2.0);
        require(solve(config), "A cap above max_sd must be accepted");

        const std::function<void(params::RDAParams&)> invalid[] = {
            [](auto& p) { p.set_min_sd(0.7); },
            [](auto& p) { p.set_min_sd(-0.1); },
            [](auto& p) { p.set_max_sd(std::numeric_limits<double>::quiet_NaN()); },
            [](auto& p) { p.set_endpoint_position_tolerance(-0.1); },
            [](auto& p) { p.set_endpoint_heading_tolerance(-0.1); },
            [](auto& p) { p.set_safety_distance_step(-0.1); },
            [](auto& p) { p.set_safety_distance_cap(-0.1); },
            [](auto& p) { p.set_safety_distance_persist(0); },
        };
        for (const auto& mutate : invalid) {
            auto bad = config;
            mutate(bad);
            require(!solve(bad), "Invalid RDA parameters must be rejected before solving");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
