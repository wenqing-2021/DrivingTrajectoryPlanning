#include "planner/trajectory_planner.h"
#include "logger/logger.h"
namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param) {
    // Constructor
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);
    pwj_speed_ptr_     = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params());
    vehicle_param_ptr_ = std::make_shared<kinematic_model::VehicleParam>(vehicle_param);
};

bool TrajPlanner::Process(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec) {
    // 1. frontend plan
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

    // 2. backend plan
    LOG(INFO) << "Start to optimize the speed profile...";
    const std::vector<Eigen::Vector3d>* const frontend_path = hybrid_astar_ptr_->GetPath();
    if (pwj_speed_ptr_->Optimize(*frontend_path, start_vec)) {
        LOG(INFO) << "The speed profile has been optimized...";
    } else {
        LOG(WARNING) << "Failed to optimize the speed profile...";
        return false;
    }

    // 3. generate the trajectory
    LOG(INFO) << "Start to generate the init trajectory status and controls...";


    return true;
};

bool TrajPlanner::setStatusControls(const std::vector<Eigen::Vector3d>* const init_path_ptr,
                                    const std::vector<Eigen::Vector3d>* const init_traj_ptr) {
    // 1. set the init status
    if (init_path_ptr->size() < 2) {
        LOG(WARNING) << "The init path size is less than 2";
        return false;
    } else if (init_path_ptr->size() != init_traj_ptr->size()) {
        LOG(WARNING) << "The init path size is less than 2";
        return false;
    }
    init_states_.resize(init_path_ptr->size(), vehicle_model::KinematicModel::kStateSize);
    init_controls_.resize(init_path_ptr->size() - 1, vehicle_model::KinematicModel::kControlSize);
    for (std::size_t i = 0; i < init_path_ptr->size(); ++i) {
        init_states_(i, 0) = (*init_path_ptr)[i].x();   // x
        init_states_(i, 1) = (*init_path_ptr)[i].y();   // y
        init_states_(i, 2) = (*init_path_ptr)[i].z();   // theta
        init_states_(i, 3) = (*init_traj_ptr)[i].z();   // v
    }

    // 2. set the init controls using the forward difference
    const double dt = pwj_speed_ptr_->GetDt();
    const double L  = vehicle_param_ptr_->length();
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