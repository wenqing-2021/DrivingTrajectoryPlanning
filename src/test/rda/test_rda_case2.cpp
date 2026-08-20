/*
 * Reproduction test for the RDA planner on BenchmarkCases/Case2.csv
 * (diagonal parking scenario). Verifies that the ADMM iteration converges and
 * the optimized trajectory is collision-free.
 */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "collision_check/base_check.h"
#include "collision_check/gjk_check.h"
#include "logger.h"
#include "map.h"
#include "math/polygon2d.h"
#include "math/vec2d.h"
#include "planner/hybrid_a_star/hybrid_a_star.h"
#include "planner/rda/rda_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include "yaml-cpp/yaml.h"

using namespace planning;

namespace {

params::RDAParams LoadRDAParamsFromYaml(const std::string& yaml_path) {
    params::RDAParams p;
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
        if (node) {
            if (node["max_iter"]) p.set_max_iter(node["max_iter"].as<int>());
            if (node["convergence_tolerance"]) p.set_convergence_tolerance(node["convergence_tolerance"].as<double>());
            if (node["penalty_weight"]) p.set_penalty_weight(node["penalty_weight"].as<double>());
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
        }
    }
    catch (const std::exception& e) {
        LOG(WARNING) << "YAML load failed: " << e.what();
    }
    return p;
}

void SaveStatesToCSV(const std::string& filename, const std::vector<Eigen::Vector4d>& states) {
    std::ofstream file(filename);
    file << "x,y,theta,v\n";
    for (const auto& s : states) {
        file << std::fixed << std::setprecision(6) << s(0) << "," << s(1) << "," << s(2) << "," << s(3) << "\n";
    }
}

void SavePolygonToCSV(const std::string& filename, const common::math::Polygon2d& polygon) {
    std::ofstream file(filename);
    file << "x,y\n";
    const auto& points = polygon.points();
    for (const auto& p : points) { file << p.x() << "," << p.y() << "\n"; }
    if (!points.empty()) { file << points[0].x() << "," << points[0].y() << "\n"; }
}

double MinMarginToObstacle(const common::math::Polygon2d& obstacle, double px, double py) {
    const auto segments   = obstacle.line_segments();
    const auto center     = obstacle.center();
    double     min_margin = std::numeric_limits<double>::max();
    for (const auto& seg : segments) {
        const auto          dir = seg.unit_direction();
        common::math::Vec2d normal(-dir.y(), dir.x());
        common::math::Vec2d to_center(center.x() - seg.start().x(), center.y() - seg.start().y());
        if (normal.InnerProd(to_center) > 0) { normal *= -1; }
        double b      = normal.InnerProd(seg.start());
        double margin = b - (normal.x() * px + normal.y() * py);
        min_margin    = std::min(min_margin, margin);
    }
    return min_margin;
}

}   // namespace

int main() {
    LOG(INFO) << "========== RDA Case2 Reproduction Test (Diagonal Parking) ==========";
    system("mkdir -p /tmp/rda_case2_results");

    try {
        // Vehicle (same as OBCA / benchmark)
        kinematic_model::VehicleParam vehicle_param;
        vehicle_param.set_length(5.0);
        vehicle_param.set_width(2.0);
        vehicle_param.set_wheel_base(2.5);
        vehicle_param.set_max_steer_angle(0.5);
        vehicle_param.set_max_acc(2.0);
        vehicle_param.set_max_velocity(5.0);
        vehicle_param.set_front_overhang(1.25);
        vehicle_param.set_rear_overhang(1.25);
        vehicle_param.set_vehicle_type("car");
        auto dynamic_model_ptr = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);

        // Case2 scenario
        problem::PlanProblem plan_problem;
        auto*                init_state = plan_problem.mutable_init_state();
        init_state->set_x(-8.85572139303482);
        init_state->set_y(0.621890547263682);
        init_state->set_theta(-0.98971402799757);
        init_state->set_v(0.0);

        auto* goal_state = plan_problem.mutable_goal_state();
        goal_state->set_x(-5.57213930348259);
        goal_state->set_y(-12.7114427860696);
        goal_state->set_theta(0.761450646475241);
        goal_state->set_v(0.0);
        *plan_problem.mutable_vehicle_param() = vehicle_param;

        // Obstacles from Case2.csv
        std::vector<std::vector<std::pair<double, double>>> obs_pts = {
            {{5.13995848718782, -25.2957187653668},
             {-5.2096245228832, -14.4381807609364},
             {-1.81555814269828, -11.2029011119882},
             {8.53402486737274, -22.0604391164186}},
            {{-7.27954112489741, -12.2666731600503},
             {-17.6291241349684, -1.40913515562},
             {-14.2350577547835, 1.8261444933282},
             {-3.88547474471248, -9.03139351110214}},
            {{-17.8563390102952, -1.912209695465},
             {1.6158082962849, -22.1293782902968},
             {-2.95591324525998, -25.2110572553382},
             {-21.4121224314967, -5.50185772067802}},
        };
        for (const auto& pts : obs_pts) {
            auto* obs = plan_problem.add_obstacle_list();
            obs->set_vertex_num(pts.size());
            for (const auto& pt : pts) {
                auto* v = obs->add_vertex_pts();
                v->set_x(pt.first);
                v->set_y(pt.second);
            }
        }
        plan_problem.set_obstacle_num(obs_pts.size());

        auto* bound = plan_problem.mutable_map_bound();
        bound->set_min_x(-25.0);
        bound->set_max_x(12.0);
        bound->set_min_y(-28.0);
        bound->set_max_y(5.0);

        auto map_ptr = std::make_shared<map::Map>(plan_problem, 0.3);
        LOG(INFO) << "Map created with " << map_ptr->GetObsList().size() << " obstacles";

        const auto& obs_list = map_ptr->GetObsList();
        for (size_t i = 0; i < obs_list.size(); ++i) {
            SavePolygonToCSV("/tmp/rda_case2_results/obstacle_" + std::to_string(i) + ".csv", obs_list[i]);
        }

        // ==================== Initial path from hybrid A* (same as production) ====
        auto collision_checker = collision_check::BaseCheck::CreateChecker<collision_check::GJKCheck>(vehicle_param);

        params::HybridAStarParams ha_params;
        ha_params.set_max_iter(20);
        ha_params.set_expand_s(1.5);
        ha_params.set_expand_step_num(5);
        ha_params.set_max_steer_angle(0.75);
        ha_params.set_search_front_num(5);
        ha_params.set_search_back_num(3);
        ha_params.set_rs_radius(15.0);
        ha_params.set_rs_step_size(0.3);
        ha_params.set_wheel_base(2.5);
        ha_params.set_node_resolution_x(0.5);
        ha_params.set_node_resolution_y(0.5);
        ha_params.set_node_resolution_theta(10.0);
        ha_params.mutable_g_cost()->set_reverse_cost(1.0);
        ha_params.mutable_g_cost()->set_head_change_cost(0.5);

        frontend::HybridAstar hybrid_astar(ha_params, map_ptr, collision_checker);

        auto start_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        start_pose_ptr->x     = init_state->x();
        start_pose_ptr->y     = init_state->y();
        start_pose_ptr->theta = init_state->theta();

        auto goal_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        goal_pose_ptr->x     = goal_state->x();
        goal_pose_ptr->y     = goal_state->y();
        goal_pose_ptr->theta = goal_state->theta();

        LOG(INFO) << "Running hybrid A* frontend...";
        if (!hybrid_astar.Plan(*start_pose_ptr, *goal_pose_ptr)) {
            LOG(ERROR) << "Hybrid A* failed to find a path!";
            return 1;
        }
        const auto* frontend_path = hybrid_astar.GetPath();
        LOG(INFO) << "Hybrid A* path found with " << frontend_path->size() << " waypoints";

        // Convert hybrid A* path (x, y, theta) to sdv_path
        vehicle_model::sdv_path init_path = backend::RDASolver::Vec3dToSdvPath(*frontend_path);

        params::RDAParams rda_params =
            LoadRDAParamsFromYaml("/root/workspace/AutomatedPark/src/config/solver_params.yaml");
        double             dt = 0.1;
        backend::RDASolver rda_solver(dynamic_model_ptr, map_ptr, rda_params, dt);

        LOG(INFO) << "Running RDA optimization...";
        bool success = rda_solver.Process(init_path, start_pose_ptr, goal_pose_ptr);
        if (!success) {
            LOG(ERROR) << "RDA solver failed!";
            return 1;
        }
        LOG(INFO) << "[PASS] RDA solver returned success";

        const auto& opt_states = rda_solver.GetStatesResult();
        SaveStatesToCSV("/tmp/rda_case2_results/optimized_trajectory.csv", opt_states);
        SaveStatesToCSV("/tmp/rda_case2_results/initial_trajectory.csv", rda_solver.GetInitialStates());

        // Verify goal
        const auto& fs      = opt_states.back();
        double      dx      = std::abs(fs(0) - goal_state->x());
        double      dy      = std::abs(fs(1) - goal_state->y());
        bool        goal_ok = (dx < 0.3 && dy < 0.3);
        LOG(INFO) << "Goal error: dx=" << dx << " dy=" << dy << " -> " << (goal_ok ? "PASS" : "FAIL");

        // Verify collision-free against all obstacles
        bool   collision_free = true;
        double worst          = std::numeric_limits<double>::lowest();
        for (const auto& s : opt_states) {
            for (const auto& obs : obs_list) {
                double m = MinMarginToObstacle(obs, s(0), s(1));
                worst    = std::max(worst, m);
                if (m >= -1e-3) { collision_free = false; }
            }
        }
        LOG(INFO) << "Worst min-margin: " << worst << " -> "
                  << (collision_free ? "PASS (collision-free)" : "FAIL (collision)");

        bool all_pass = success && goal_ok && collision_free;
        LOG(INFO) << "========== " << (all_pass ? "TEST PASSED" : "TEST FAILED") << " ==========";
        return all_pass ? 0 : 1;
    }
    catch (const std::exception& e) {
        LOG(ERROR) << "Exception: " << e.what();
        return 1;
    }
}
