// OCEAN parameter validation: an invalid configuration must be reported as an
// explicit failure instead of crashing or silently planning with bad settings.
#include "logger.h"
#include "map/map.h"
#include "params.pb.h"
#include "planner/ocean/ocean_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <iostream>
#include <memory>

using namespace planning;
namespace {
problem::PlanProblem SquareObstacleProblem() {
    problem::PlanProblem problem;
    auto*                init = problem.mutable_init_state();
    init->set_x(0);
    init->set_y(0);
    init->set_theta(0);
    auto* goal = problem.mutable_goal_state();
    goal->set_x(10);
    goal->set_y(0);
    goal->set_theta(0);
    auto* vehicle = problem.mutable_vehicle_param();
    vehicle->set_vehicle_type("car");
    vehicle->set_length(4.5);
    vehicle->set_width(2.0);
    vehicle->set_wheel_base(2.5);
    vehicle->set_front_overhang(1.0);
    vehicle->set_rear_overhang(1.0);
    vehicle->set_max_steer_angle(0.75);
    vehicle->set_max_acc(2.0);
    vehicle->set_max_velocity(15.0);
    auto* obstacle = problem.add_obstacle_list();
    obstacle->set_vertex_num(4);
    for (const auto& corner : {std::pair<double, double>{6.0, 1.5}, {8.0, 1.5}, {8.0, 3.5}, {6.0, 3.5}}) {
        auto* point = obstacle->add_vertex_pts();
        point->set_x(corner.first);
        point->set_y(corner.second);
    }
    problem.set_obstacle_num(1);
    auto* bound = problem.mutable_map_bound();
    bound->set_min_x(-5);
    bound->set_max_x(15);
    bound->set_min_y(-5);
    bound->set_max_y(5);
    return problem;
}
}   // namespace

int main() {
    const auto              problem = SquareObstacleProblem();
    const auto              map     = std::make_shared<map::Map>(problem, 0.1);
    const auto              model   = std::make_shared<vehicle_model::KinematicModel>(problem.vehicle_param());
    vehicle_model::sdv_path path;
    for (int i = 0; i < 3; ++i) path.push_back({i * 5.0, 0.0, 0.0});
    const auto start = std::make_shared<vehicle_model::VehiclePose>(vehicle_model::VehiclePose{0, 0, 0});
    const auto goal  = std::make_shared<vehicle_model::VehiclePose>(vehicle_model::VehiclePose{10, 0, 0});
    const auto run   = [&](const params::OCEANParams& tweak, const char* name) {
        try {
            const auto           workspace = std::make_shared<backend::OCEANWorkspace>();
            backend::OCEANSolver solver(model, map, tweak, 0.1, workspace);
            auto                 start_pose = std::make_shared<vehicle_model::VehiclePose>(*start);
            auto                 goal_pose  = std::make_shared<vehicle_model::VehiclePose>(*goal);
            const bool           success    = solver.Process(path, start_pose, goal_pose);
            const std::string    status     = solver.GetStats().status();
            if (success || status != "invalid_parameters") {
                LOG(ERROR) << name << ": expected invalid_parameters, got success=" << success << " status=" << status;
                return false;
            }
            return true;
        }
        catch (const std::exception& error) {
            LOG(ERROR) << name << ": threw " << error.what();
            return false;
        }
    };
    int        failures = 0;
    const auto check    = [&](const params::OCEANParams& tweak, const char* name) {
        if (!run(tweak, name)) ++failures;
    };
    params::OCEANParams base;
    base.set_rho(0.0);
    check(base, "zero_rho");
    base = params::OCEANParams();
    base.set_dt_min(0.2);
    base.set_dt_max(0.05);
    check(base, "inverted_dt_bounds");
    base = params::OCEANParams();
    base.set_endpoint_position_tolerance(0.2);
    check(base, "loose_endpoint_position_tolerance");
    base = params::OCEANParams();
    base.set_endpoint_heading_tolerance(0.5);
    check(base, "loose_endpoint_heading_tolerance");
    base = params::OCEANParams();
    base.set_workers(-1);
    check(base, "negative_workers");
    base = params::OCEANParams();
    base.set_worker_timeout_ms(0);
    check(base, "zero_worker_timeout");
    base = params::OCEANParams();
    base.set_min_clearance(-1.0);
    check(base, "negative_clearance");
    base = params::OCEANParams();
    base.set_w_distance(-1.0);
    check(base, "negative_distance_weight");
    base = params::OCEANParams();
    base.set_max_inner_iter(0);
    check(base, "zero_inner_iterations");
    base = params::OCEANParams();
    base.set_primal_tolerance(0.0);
    check(base, "zero_primal_tolerance");
    base = params::OCEANParams();
    base.set_trust_region(-0.5);
    check(base, "negative_trust_region");
    params::OCEANParams zero = params::OCEANParams();
    zero.set_min_clearance(0.0);
    zero.set_endpoint_position_tolerance(0.0);
    const auto defaults = backend::OCEANSolver::Defaults(zero);
    if (defaults.min_clearance() != 0.0 || defaults.endpoint_position_tolerance() != 0.0) {
        LOG(ERROR) << "explicit zero must survive defaulting";
        ++failures;
    }
    if (failures == 0) std::cout << "OCEAN parameter validation passed\n";
    return failures == 0 ? 0 : 1;
}
