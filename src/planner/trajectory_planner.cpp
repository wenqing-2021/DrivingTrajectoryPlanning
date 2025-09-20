#include "trajectory_planner.h"

namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param) {
    // Constructor
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);
    pwj_speed_ptr_   = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params());
    admm_solver_ptr_ = std::make_unique<admmopt>();
    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    // maximum iterations
    options += "Integer max_iter     10\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-6\n";
    // derivative tesing
    options += "String derivative_test   second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius   0.\n";
    vehicle_model_ptr_ = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);
    obca_solver_ptr_   = std::make_unique<obcaopt>(options, vehicle_model_ptr_, map_ptr);
};

bool TrajPlanner::Process(const vehicle_model::VehiclePose& start_vec, const vehicle_model::VehiclePose& goal_vec) {
    // 1. frontend path plan
    LOG(INFO) << "Start to search the frontend path...";
    if (hybrid_astar_ptr_->Plan(start_vec, goal_vec)) {
        LOG(INFO) << "The frontend path is found...";
        hybrid_astar_ptr_->GetPathLength(frontend_path_length_);
    } else {
        LOG(WARNING) << "Failed to find the frontend path...";
        debug_node_list_ = hybrid_astar_ptr_->GetDebugNodeList();
        LOG(INFO) << "The debug node list size is: " << debug_node_list_.size();
        return false;
    }

    // 2. backend pwj speed plan
    LOG(INFO) << "Start to optimize the speed profile...";
    const std::vector<Eigen::Vector3d>* const frontend_path = hybrid_astar_ptr_->GetPath();
    if (pwj_speed_ptr_->Optimize(*frontend_path, start_vec)) {
        LOG(INFO) << "The speed profile has been optimized...";
    } else {
        LOG(WARNING) << "Failed to optimize the speed profile...";
        return false;
    }

    // 3. generate the trajectory with TrajOptimizer
    LOG(INFO) << "Start to generate the init trajectory status and controls...";
    if (!setStatusControls(frontend_path, pwj_speed_ptr_->GetResult())) { return false; }
    LOG(INFO) << "The init trajectory status and controls have been set...";

    // 4. solve the trajectory optimization problem with OBCA
    LOG(INFO) << "Start to optimize the trajectory with OBCA...";

    return true;
};

bool TrajPlanner::setStatusControls(const std::vector<Eigen::Vector3d>* const init_path_ptr,
                                    const std::vector<Eigen::Vector3d>* const init_traj_ptr) {
    // 1. set the init status
    if (init_path_ptr->size() < 2) {
        LOG(WARNING) << "The init path size is less than 2";
        return false;
    } else if (init_path_ptr->size() != init_traj_ptr->size()) {
        LOG(WARNING) << "The init path size is not equal to the init traj size"
                     << "| path size is" << init_path_ptr->size() << " | traj size is " << init_traj_ptr->size();
        return false;
    }
    init_states_.resize(init_path_ptr->size(), vehicle_model::KinematicModel::GetStateSize());
    init_controls_.resize(init_path_ptr->size() - 1, vehicle_model::KinematicModel::GetControlSize());
    for (std::size_t i = 0; i < init_path_ptr->size(); ++i) {
        init_states_(i, 0) = (*init_path_ptr)[i].x();   // x
        init_states_(i, 1) = (*init_path_ptr)[i].y();   // y
        init_states_(i, 2) = (*init_path_ptr)[i].z();   // theta
        init_states_(i, 3) = (*init_traj_ptr)[i].z();   // v
    }

    // 2. set the init controls using the forward difference
    const double dt = pwj_speed_ptr_->GetDt();
    const double L  = vehicle_model_ptr_->GetVehicleParam().length();
    for (std::size_t i = 0; i < init_path_ptr->size() - 1; ++i) {
        init_controls_(i, 0) = ((*init_traj_ptr)[i + 1].z() - (*init_traj_ptr)[i].z()) / dt;   // a
        // compute steer angle
        double delta_theta = common::math::NormalizeAngle((*init_path_ptr)[i + 1].z() - (*init_path_ptr)[i].z());
        double delta_x     = (*init_path_ptr)[i + 1].x() - (*init_path_ptr)[i].x();
        double delta_y     = (*init_path_ptr)[i + 1].y() - (*init_path_ptr)[i].y();
        double delta_s     = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (std::abs(delta_s) < kEpsilon) {
            init_controls_(i, 1) = 0.0;
        } else {
            init_controls_(i, 1) = std::atan2(L * delta_theta, delta_s);
            init_controls_(i, 1) = common::math::NormalizeAngle(init_controls_(i, 1));
        }
    }

    return true;
};

}   // namespace planning