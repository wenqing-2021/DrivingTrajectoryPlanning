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
#include "obca_planner.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"

using namespace planning;

// Helper function to save vector to CSV
void SaveVectorToCSV(const std::string& filename, const std::vector<Eigen::Vector3d>& data, const std::string& header) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return;
    }

    file << header << "\n";
    for (const auto& point : data) {
        file << std::fixed << std::setprecision(6) << point(0) << "," << point(1) << "," << point(2) << "\n";
    }
    file.close();
    LOG(INFO) << "Saved " << data.size() << " points to " << filename;
}

// Helper function to save states to CSV
void SaveStatesToCSV(const std::string& filename, const std::vector<Eigen::Vector4d>& states,
                     const std::string& header) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return;
    }

    file << header << "\n";
    for (const auto& state : states) {
        file << std::fixed << std::setprecision(6) << state(0) << "," << state(1) << "," << state(2) << "," << state(3)
             << "\n";
    }
    file.close();
    LOG(INFO) << "Saved " << states.size() << " states to " << filename;
}

// Helper function to save controls to CSV
void SaveControlsToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& controls,
                       const std::string& header) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return;
    }

    file << header << "\n";
    for (const auto& control : controls) {
        file << std::fixed << std::setprecision(6) << control(0) << "," << control(1) << "\n";
    }
    file.close();
    LOG(INFO) << "Saved " << controls.size() << " controls to " << filename;
}

// Helper function to save polygon to CSV
void SavePolygonToCSV(const std::string& filename, const common::math::Polygon2d& polygon, const std::string& header) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << filename;
        return;
    }

    file << header << "\n";
    const auto& points = polygon.points();
    for (const auto& point : points) {
        file << std::fixed << std::setprecision(6) << point.x() << "," << point.y() << "\n";
    }
    // Close the polygon by adding the first point again
    if (!points.empty()) {
        file << std::fixed << std::setprecision(6) << points[0].x() << "," << points[0].y() << "\n";
    }
    file.close();
    LOG(INFO) << "Saved polygon to " << filename;
}

int main() {
    LOG(INFO) << "========== OBCA Planner Test ==========";

    // Create output directory for results
    system("mkdir -p /tmp/obca_test_results");

    try {
        // ==================== Setup Problem ====================
        LOG(INFO) << "Setting up problem...";

        // Create vehicle parameters
        kinematic_model::VehicleParam vehicle_param;
        vehicle_param.set_length(5.0);            // 2.0 m
        vehicle_param.set_width(2.0);             // 1.0 m
        vehicle_param.set_wheel_base(2.5);        // 2.5 m
        vehicle_param.set_max_steer_angle(0.5);   // 0.5 rad
        vehicle_param.set_max_acc(2.0);           // 2.0 m/s^2
        vehicle_param.set_max_velocity(5.0);      // 5.0 m/s
        vehicle_param.set_front_overhang(1.25);   // 1.25 m
        vehicle_param.set_rear_overhang(1.25);    // 1.25 m
        vehicle_param.set_vehicle_type("car");

        // Create kinematic model
        auto dynamic_model_ptr = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);
        LOG(INFO) << "Vehicle created: length=" << vehicle_param.length() << "m, width=" << vehicle_param.width()
                  << "m";

        // Create problem definition
        problem::PlanProblem plan_problem;

        // Set initial state (start from left)
        kinematic_model::StateVar* init_state = plan_problem.mutable_init_state();
        init_state->set_x(0.0);
        init_state->set_y(0.0);
        init_state->set_theta(0.0);
        init_state->set_v(0.0);

        // Set goal state (end at right)
        kinematic_model::StateVar* goal_state = plan_problem.mutable_goal_state();
        goal_state->set_x(30.0);
        goal_state->set_y(0.0);
        goal_state->set_theta(0.0);
        goal_state->set_v(0.0);

        LOG(INFO) << "Start position: (" << init_state->x() << ", " << init_state->y() << ")";
        LOG(INFO) << "Goal position: (" << goal_state->x() << ", " << goal_state->y() << ")";

        // Set vehicle parameters
        *plan_problem.mutable_vehicle_param() = vehicle_param;

        // Create a square obstacle above the path
        // Square with corners at (4, 0.5) to (6, 1.5)
        // Initial path (y=0) doesn't pass through it, but vehicle body (width=2m) will collide
        problem::Polygon* obs_polygon = plan_problem.add_obstacle_list();
        obs_polygon->set_vertex_num(4);

        // Bottom-left
        cost_map::Pos2D* vertex1 = obs_polygon->add_vertex_pts();
        vertex1->set_x(11.0);
        vertex1->set_y(2.7);

        // Bottom-right
        cost_map::Pos2D* vertex2 = obs_polygon->add_vertex_pts();
        vertex2->set_x(13.0);
        vertex2->set_y(2.7);

        // Top-right
        cost_map::Pos2D* vertex3 = obs_polygon->add_vertex_pts();
        vertex3->set_x(13.0);
        vertex3->set_y(3.7);

        // Top-left
        cost_map::Pos2D* vertex4 = obs_polygon->add_vertex_pts();
        vertex4->set_x(11.0);
        vertex4->set_y(3.7);

        plan_problem.set_obstacle_num(1);
        LOG(INFO) << "Obstacle created: Square from (11, 0.7) to (13, 1.7)";

        // Set map bounds
        problem::MapBound* map_bound = plan_problem.mutable_map_bound();
        map_bound->set_min_x(-1.0);
        map_bound->set_max_x(31.0);
        map_bound->set_min_y(-2.0);
        map_bound->set_max_y(3.0);

        // Create map
        double map_resolution = 0.1;
        auto   map_ptr        = std::make_shared<map::Map>(plan_problem, map_resolution);
        LOG(INFO) << "Map created with resolution " << map_resolution << "m";

        // Save initial path and obstacles for visualization
        {
            LOG(INFO) << "Saving initial problem data...";

            // Save start and goal positions
            std::vector<Eigen::Vector3d> start_goal;
            start_goal.push_back(Eigen::Vector3d(init_state->x(), init_state->y(), init_state->theta()));
            start_goal.push_back(Eigen::Vector3d(goal_state->x(), goal_state->y(), goal_state->theta()));
            SaveVectorToCSV("/tmp/obca_test_results/start_goal.csv", start_goal, "x,y,theta");

            // Save obstacles
            const auto& obs_list = map_ptr->GetObsList();
            for (size_t i = 0; i < obs_list.size(); ++i) {
                std::string filename = "/tmp/obca_test_results/obstacle_" + std::to_string(i) + ".csv";
                SavePolygonToCSV(filename, obs_list[i], "x,y");
            }
        }

        // ==================== Create Initial Path ====================
        LOG(INFO) << "Creating initial path...";
        vehicle_model::sdv_path init_path;

        // Generate initial path: 60 waypoints from (0, 0) to (30, 0)
        for (int i = 0; i < 60; ++i) {
            vehicle_model::VehiclePose pose;
            pose.x     = i * (30.0 / 59.0);
            pose.y     = 0.0;
            pose.theta = 0.0;
            init_path.push_back(pose);
        }
        LOG(INFO) << "Initial path created with " << init_path.size() << " waypoints";

        // Save initial path
        {
            std::vector<Eigen::Vector3d> init_path_points;
            for (const auto& pose : init_path) {
                init_path_points.push_back(Eigen::Vector3d(pose.x, pose.y, pose.theta));
            }
            SaveVectorToCSV("/tmp/obca_test_results/initial_path.csv", init_path_points, "x,y,theta");
        }

        // Create start and goal poses
        auto start_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        start_pose_ptr->x     = 0.0;
        start_pose_ptr->y     = 0.0;
        start_pose_ptr->theta = 0.0;

        auto goal_pose_ptr   = std::make_shared<vehicle_model::VehiclePose>();
        goal_pose_ptr->x     = 30.0;
        goal_pose_ptr->y     = 0.0;
        goal_pose_ptr->theta = 0.0;

        // ==================== Configure Solver ====================
        LOG(INFO) << "Configuring IPOPT solver...";

        std::string ipopt_options;
        // ipopt_options += "Integer print_level         5\n";
        ipopt_options += "String  sb                  yes\n";
        ipopt_options += "Integer max_iter            20\n";
        ipopt_options += "Numeric tol                 1e-3\n";
        // Convergence based on objective function change
        // ipopt_options += "String derivative_test   second-order\n";

        LOG(INFO) << "IPOPT configured";

        // ==================== Run OBCA Solver ====================
        LOG(INFO) << "Creating OBCA solver...";
        params::OBCAParams obca_params;
        obca_params.set_ref_weight(5.0);
        obca_params.set_smooth_weight(0.5);
        obca_params.set_control_weight(100.0);
        backend::OBCASolver obca_solver(ipopt_options, dynamic_model_ptr, map_ptr, obca_params);
        LOG(INFO) << "OBCA solver created";

        LOG(INFO) << "Running OBCA optimization...";
        bool success = obca_solver.Process(init_path, start_pose_ptr, goal_pose_ptr);

        if (!success) {
            LOG(ERROR) << "OBCA solver failed to find a solution!";
            return 1;
        }

        LOG(INFO) << "✓ OBCA solver succeeded!";

        // ==================== Get Results ====================
        LOG(INFO) << "Extracting results...";
        const auto& states   = obca_solver.GetStatesResult();
        const auto& controls = obca_solver.GetControlsResult();

        LOG(INFO) << "Number of states: " << states.size();
        LOG(INFO) << "Number of controls: " << controls.size();

        // Save results to CSV
        {
            LOG(INFO) << "Saving results to CSV...";
            SaveStatesToCSV("/tmp/obca_test_results/optimized_trajectory.csv", states, "x,y,theta,v");
            SaveControlsToCSV("/tmp/obca_test_results/optimized_controls.csv", controls, "acceleration,steer_angle");
        }

        // Print some results
        LOG(INFO) << "First state: x=" << states[0](0) << " y=" << states[0](1) << " theta=" << states[0](2)
                  << " v=" << states[0](3);
        LOG(INFO) << "Last state: x=" << states.back()(0) << " y=" << states.back()(1) << " theta=" << states.back()(2)
                  << " v=" << states.back()(3);

        // Check the final state is close to the goal
        const auto& final_state = states.back();
        double      x_error     = std::abs(final_state(0) - goal_pose_ptr->x);
        double      y_error     = std::abs(final_state(1) - goal_pose_ptr->y);

        LOG(INFO) << "Final position error: dx=" << x_error << "m, dy=" << y_error << "m";

        if (x_error < 0.2 && y_error < 0.2) {
            LOG(INFO) << "✓ Test PASSED: Final position close to goal";
        } else {
            LOG(WARNING) << "⚠ Test WARNING: Final position may be far from goal";
        }

        LOG(INFO) << "========== Test Complete ==========";
        LOG(INFO) << "Results saved to: /tmp/obca_test_results/";

        return 0;
    }
    catch (const std::exception& e) {
        LOG(ERROR) << "Exception caught: " << e.what();
        return 1;
    }
}
