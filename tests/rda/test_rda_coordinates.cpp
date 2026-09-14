#include "planner/rda/rda_planner.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

using planning::vehicle_model::VehiclePose;

std::vector<Eigen::Vector4d> run(double offset_x, double offset_y, double turns) {
    kinematic_model::VehicleParam vehicle;
    vehicle.set_length(4.5); vehicle.set_width(2.0); vehicle.set_wheel_base(2.5);
    vehicle.set_rear_overhang(1.0); vehicle.set_front_overhang(1.0);
    vehicle.set_max_velocity(15.0); vehicle.set_max_acc(2.0); vehicle.set_max_steer_angle(0.7);
    planning::vehicle_model::sdv_path path;
    double x = 0.0, y = 0.0, theta = 3.0;
    for (int t = 0; t <= 60; ++t) {
        VehiclePose pose;
        pose.x = x + offset_x; pose.y = y + offset_y;
        pose.theta = std::atan2(std::sin(theta), std::cos(theta));
        path.push_back(pose);
        x -= 0.1 * std::cos(theta); y -= 0.1 * std::sin(theta); theta += 0.007;
    }
    auto start = std::make_shared<VehiclePose>(path.front());
    auto goal = std::make_shared<VehiclePose>(path.back());
    start->theta += turns; goal->theta += turns;
    const auto original_start = *start;
    problem::PlanProblem problem;
    *problem.mutable_vehicle_param() = vehicle;
    auto* b = problem.mutable_map_bound();
    b->set_min_x(offset_x - 5); b->set_max_x(offset_x + 12);
    b->set_min_y(offset_y - 5); b->set_max_y(offset_y + 5);
    auto model = std::make_shared<planning::vehicle_model::KinematicModel>(vehicle);
    auto map = std::make_shared<map::Map>(problem, 0.3);
    params::RDAParams params;
    params.set_max_iter(5); params.set_convergence_tolerance(0.05);
    params.set_dt_min(0.05); params.set_dt_max(0.2);
    planning::backend::RDASolver solver(model, map, params, 0.1);
    if (!solver.Process(path, start, goal)) throw std::runtime_error("Reverse curved path failed");
    if (start->x != original_start.x || start->theta != original_start.theta)
        throw std::runtime_error("Input poses were mutated");
    auto result = solver.GetStatesResult();
    bool reverse = false;
    for (auto& p : result) {
        p.x() -= offset_x; p.y() -= offset_y;
        reverse |= p.w() < -0.1;
    }
    if (!reverse || std::hypot(result.back().x() - (goal->x - offset_x),
                                result.back().y() - (goal->y - offset_y)) > 0.15)
        throw std::runtime_error("Reverse velocity or world-coordinate output is invalid");
    return result;
}

int main() {
    try {
        const auto baseline = run(0, 0, 0);
        const auto shifted = run(4.5e9, -5.5e9, -2 * std::acos(-1.0));
        if (baseline.size() != shifted.size()) return 1;
        for (std::size_t i = 0; i < baseline.size(); ++i) {
            if ((baseline[i] - shifted[i]).norm() > 0.01) {
                std::cerr << "Translation/angle invariance failed at " << i << '\n';
                return 2;
            }
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 3;
    }
}
