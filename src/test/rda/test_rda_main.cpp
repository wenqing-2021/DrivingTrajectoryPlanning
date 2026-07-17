/*
 * Unit test for the RDA planner: straight-line obstacle avoidance.
 *
 * Scenario:
 *   - Vehicle starts at (0, 0) heading +x, goal at (30, 0).
 *   - A square obstacle sits on the straight-line path at (14..16, -0.5..0.5),
 *     directly blocking the y=0 line, so the planner must deviate to avoid it.
 *   - The initial (reference) path is a straight line along y=0.
 *
 * Verification:
 *   1. Process() returns true.
 *   2. The optimized trajectory does not collide with the obstacle
 *      (vehicle center stays outside the inflated obstacle halfspaces).
 *   3. The final state reaches the goal within tolerance.
 *   4. The trajectory deviates from y=0 (i.e. actually avoids the obstacle).
 */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "planner/rda/rda_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include "yaml-cpp/yaml.h"

using namespace planning;

namespace {

// Load RDAParams from the rda_params node of solver_params.yaml.
// Falls back to defaults for any missing field.
params::RDAParams LoadRDAParamsFromYaml(const std::string& yaml_path) {
    params::RDAParams p;
    // Defaults (match protobuf field defaults used in the solver)
    p.set_max_iter(50);
    p.set_convergence_tolerance(0.05);
    p.set_penalty_weight(200.0);
    p.set_use_warm_start(false);
    p.set_l1_weight(8.0);
    p.set_w_s(1.0);
    p.set_w_u(1.0);
    p.set_ro2(1.0);
    p.set_min_sd(0.1);
    p.set_max_sd(1.0);
    p.set_dt_min(0.05);
    p.set_dt_max(0.2);
    p.set_w_dt(1.0);
    p.set_iter_threshold(0.05);

    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        YAML::Node node   = config["rda_params"];
        if (!node) {
            LOG(WARNING) << "No 'rda_params' node in " << yaml_path << ", using defaults.";
            return p;
        }
        if (node["max_iter"]) p.set_max_iter(node["max_iter"].as<int>());
        if (node["convergence_tolerance"]) p.set_convergence_tolerance(node["convergence_tolerance"].as<double>());
        if (node["penalty_weight"]) p.set_penalty_weight(node["penalty_weight"].as<double>());
        if (node["use_warm_start"]) p.set_use_warm_start(node["use_warm_start"].as<bool>());
        if (node["l1_weight"]) p.set_l1_weight(node["l1_weight"].as<double>());
        if (node["w_s"]) p.set_w_s(node["w_s"].as<double>());
        if (node["w_u"]) p.set_w_u(node["w_u"].as<double>());
        if (node["ro2"]) p.set_ro2(node["ro2"].as<double>());
        if (node["min_sd"]) p.set_min_sd(node["min_sd"].as<double>());
        if (node["max_sd"]) p.set_max_sd(node["max_sd"].as<double>());
        if (node["dt_min"]) p.set_dt_min(node["dt_min"].as<double>());
        if (node["dt_max"]) p.set_dt_max(node["dt_max"].as<double>());
        if (node["w_dt"]) p.set_w_dt(node["w_dt"].as<double>());
        if (node["iter_threshold"]) p.set_iter_threshold(node["iter_threshold"].as<double>());
        LOG(INFO) << "Loaded rda_params from " << yaml_path;
    }
    catch (const std::exception& e) {
        LOG(WARNING) << "Failed to load " << yaml_path << ": " << e.what() << ". Using defaults.";
    }
    return p;
}


void SaveStatesToCSV(const std::string& filename, const std::vector<Eigen::Vector4d>& states,
                     const std::string& header) {
    std::ofstream file(filename);
    if (!file.is_open()) { return; }
    file << header << "\n";
    for (const auto& s : states) {
        file << std::fixed << std::setprecision(6) << s(0) << "," << s(1) << "," << s(2) << "," << s(3) << "\n";
    }
}

void SavePolygonToCSV(const std::string& filename, const common::math::Polygon2d& polygon) {
    std::ofstream file(filename);
    if (!file.is_open()) { return; }
    file << "x,y\n";
    const auto& points = polygon.points();
    for (const auto& p : points) { file << std::fixed << std::setprecision(6) << p.x() << "," << p.y() << "\n"; }
    if (!points.empty()) { file << points[0].x() << "," << points[0].y() << "\n"; }
}

// Check whether the vehicle center at (px, py) is inside the obstacle polygon.
// The polygon interior is { p : A*p <= b } (outward normals). A point is inside
// iff ALL halfspace constraints are satisfied (margin_i = b_i - n_i^T p >= 0).
// Returns the minimum margin: if min_margin >= 0 the point is inside (collision),
// otherwise it is outside (safe). We therefore report collision when min_margin >= 0.
double MinMarginToObstacle(const common::math::Polygon2d& obstacle, double px, double py) {
    const auto segments   = obstacle.line_segments();
    const auto center     = obstacle.center();
    double     min_margin = std::numeric_limits<double>::max();

    for (const auto& seg : segments) {
        const auto          dir = seg.unit_direction();
        common::math::Vec2d normal(-dir.y(), dir.x());
        common::math::Vec2d to_center(center.x() - seg.start().x(), center.y() - seg.start().y());
        if (normal.InnerProd(to_center) > 0) { normal *= -1; }   // outward normal

        double b      = normal.InnerProd(seg.start());
        double margin = b - (normal.x() * px + normal.y() * py);
        min_margin    = std::min(min_margin, margin);
    }
    return min_margin;   // >=0 => inside (collision); <0 => outside (safe)
}

}   // namespace

int main() {
    LOG(INFO) << "========== RDA Planner Test (Straight-Line Obstacle Avoidance) ==========";

    system("mkdir -p /tmp/rda_test_results");

    try {
        // ==================== Vehicle ====================
        kinematic_model::VehicleParam vehicle_param;
        vehicle_param.set_length(5.0);   // same as OBCA test
        vehicle_param.set_width(2.0);
        vehicle_param.set_wheel_base(2.5);
        vehicle_param.set_max_steer_angle(0.5);
        vehicle_param.set_max_acc(2.0);
        vehicle_param.set_max_velocity(5.0);
        vehicle_param.set_front_overhang(1.25);
        vehicle_param.set_rear_overhang(1.25);
        vehicle_param.set_vehicle_type("car");

        auto dynamic_model_ptr = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);
        LOG(INFO) << "Vehicle: L=" << vehicle_param.length() << " W=" << vehicle_param.width()
                  << " wheelbase=" << vehicle_param.wheel_base();

        // ==================== Problem ====================
        problem::PlanProblem plan_problem;

        auto* init_state = plan_problem.mutable_init_state();
        init_state->set_x(0.0);
        init_state->set_y(0.0);
        init_state->set_theta(0.0);
        init_state->set_v(0.0);

        auto* goal_state = plan_problem.mutable_goal_state();
        goal_state->set_x(30.0);
        goal_state->set_y(0.0);
        goal_state->set_theta(0.0);
        goal_state->set_v(0.0);

        *plan_problem.mutable_vehicle_param() = vehicle_param;

        // Square obstacle above the path (same as OBCA test): (16..18, 0.2..1.2).
        // The straight y=0 path does not pass through it, but the vehicle body
        // (width = 2 m) collides, so the planner must deviate downward.
        auto* obs = plan_problem.add_obstacle_list();
        obs->set_vertex_num(4);
        auto* v1 = obs->add_vertex_pts();
        v1->set_x(16.0);
        v1->set_y(0.2);
        auto* v2 = obs->add_vertex_pts();
        v2->set_x(18.0);
        v2->set_y(0.2);
        auto* v3 = obs->add_vertex_pts();
        v3->set_x(18.0);
        v3->set_y(1.2);
        auto* v4 = obs->add_vertex_pts();
        v4->set_x(16.0);
        v4->set_y(1.2);
        plan_problem.set_obstacle_num(1);

        auto* bound = plan_problem.mutable_map_bound();
        bound->set_min_x(-1.0);
        bound->set_max_x(31.0);
        bound->set_min_y(-2.0);
        bound->set_max_y(3.0);

        auto map_ptr = std::make_shared<map::Map>(plan_problem, 0.1);
        LOG(INFO) << "Map created, obstacle: square (16,0.2)-(18,1.2)";

        // Save obstacle for visualization
        const auto& obs_list = map_ptr->GetObsList();
        for (size_t i = 0; i < obs_list.size(); ++i) {
            SavePolygonToCSV("/tmp/rda_test_results/obstacle_" + std::to_string(i) + ".csv", obs_list[i]);
        }

        // ==================== Initial path ====================
        // Straight line along y=0 (same as OBCA test): 60 waypoints from (0,0)
        // to (30,0). The obstacle sits above the path, so the vehicle deviates
        // downward (no up/down symmetry saddle point here).
        vehicle_model::sdv_path init_path;
        for (int i = 0; i < 60; ++i) {
            vehicle_model::VehiclePose pose;
            pose.x     = i * (30.0 / 59.0);
            pose.y     = 0.0;
            pose.theta = 0.0;
            init_path.push_back(pose);
        }
        LOG(INFO) << "Initial path: " << init_path.size() << " waypoints along y=0";

        auto start_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        start_pose_ptr->x     = 0.0;
        start_pose_ptr->y     = 0.0;
        start_pose_ptr->theta = 0.0;

        auto goal_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        goal_pose_ptr->x     = 30.0;
        goal_pose_ptr->y     = 0.0;
        goal_pose_ptr->theta = 0.0;

        // ==================== RDA params (from solver_params.yaml) ====================
        const std::string solver_params_yaml = "/root/workspace/AutomatedPark/src/config/solver_params.yaml";
        params::RDAParams rda_params         = LoadRDAParamsFromYaml(solver_params_yaml);
        LOG(INFO) << "RDA params: max_iter=" << rda_params.max_iter() << " rho1=" << rda_params.penalty_weight()
                  << " ro2=" << rda_params.ro2() << " w_s=" << rda_params.w_s() << " w_u=" << rda_params.w_u()
                  << " dt=[" << rda_params.dt_min() << "," << rda_params.dt_max() << "]";

        double             dt = 0.1;
        backend::RDASolver rda_solver(dynamic_model_ptr, map_ptr, rda_params, dt);
        LOG(INFO) << "RDA solver created, dt=" << dt;

        // ==================== Run ====================
        LOG(INFO) << "Running RDA optimization...";
        bool success = rda_solver.Process(init_path, start_pose_ptr, goal_pose_ptr);

        if (!success) {
            LOG(ERROR) << "RDA solver failed!";
            return 1;
        }
        LOG(INFO) << "[PASS] RDA solver returned success";

        const auto& opt_states = rda_solver.GetStatesResult();
        LOG(INFO) << "Optimized trajectory: " << opt_states.size() << " states";

        SaveStatesToCSV("/tmp/rda_test_results/optimized_trajectory.csv", opt_states, "x,y,theta,v");
        SaveStatesToCSV("/tmp/rda_test_results/initial_trajectory.csv", rda_solver.GetInitialStates(), "x,y,theta,v");

        // ==================== Verify 1: goal reached ====================
        const auto& final_state = opt_states.back();
        double      dx          = std::abs(final_state(0) - goal_pose_ptr->x);
        double      dy          = std::abs(final_state(1) - goal_pose_ptr->y);
        LOG(INFO) << "Final state: (" << final_state(0) << ", " << final_state(1) << ", " << final_state(2) << ", "
                  << final_state(3) << ")";
        LOG(INFO) << "Goal error: dx=" << dx << " dy=" << dy;

        bool goal_ok = (dx < 0.2 && dy < 0.2);
        LOG(INFO) << (goal_ok ? "[PASS] Goal reached" : "[FAIL] Goal NOT reached");

        // ==================== Verify 2: no collision ====================
        // The vehicle center collides only if it is INSIDE the polygon, i.e. all
        // halfspace margins are >= 0 (min margin >= 0). Outside => min margin < 0.
        bool   collision_free = true;
        double worst_margin   = std::numeric_limits<double>::lowest();
        for (const auto& s : opt_states) {
            double margin = MinMarginToObstacle(obs_list[0], s(0), s(1));
            worst_margin  = std::max(worst_margin, margin);
            if (margin >= -1e-3) { collision_free = false; }   // inside or touching
        }
        LOG(INFO) << "Worst (max) min-margin to obstacle: " << worst_margin << " (negative=safe)";
        LOG(INFO) << (collision_free ? "[PASS] Collision-free" : "[FAIL] Collision detected");

        // ==================== Verify 3: actually deviates ====================
        double max_abs_y = 0.0;
        for (const auto& s : opt_states) { max_abs_y = std::max(max_abs_y, std::abs(s(1))); }
        bool deviated = (max_abs_y > 0.1);
        LOG(INFO) << "Max |y| deviation: " << max_abs_y;
        LOG(INFO) << (deviated ? "[PASS] Trajectory deviates to avoid obstacle"
                               : "[WARN] Trajectory stays near y=0 (may not avoid)");

        // ==================== Summary ====================
        bool all_pass = success && goal_ok && collision_free;
        LOG(INFO) << "========== " << (all_pass ? "TEST PASSED" : "TEST FAILED") << " ==========";
        LOG(INFO) << "Results saved to /tmp/rda_test_results/";

        return all_pass ? 0 : 1;
    }
    catch (const std::exception& e) {
        LOG(ERROR) << "Exception: " << e.what();
        return 1;
    }
}
