#include "planner/trajectory_planner.h"
#include "logger/logger.h"
namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker) {
    // Constructor
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);
    pwj_speed_ptr_ = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params());
};

bool TrajPlanner::Process(const Eigen::Vector3d& start_vec, const Eigen::Vector3d& goal_vec) {
    // 1. frontend plan
    LOG(INFO) << "Start to search the frontend path...";
    if (hybrid_astar_ptr_->Plan(start_vec, goal_vec)) {
        LOG(INFO) << "The frontend path is found...";
        hybrid_astar_ptr_->GetPath(frontend_path_, frontend_path_length_);
    } else {
        LOG(WARNING) << "Failed to find the frontend path...";
        debug_node_list_ = hybrid_astar_ptr_->GetDebugNodeList();
        LOG(INFO) << "The debug node list size is: " << debug_node_list_.size();
        return false;
    }

    // 2. backend plan
    LOG(INFO) << "Start to optimize the speed profile...";
    if (pwj_speed_ptr_->Optimize(frontend_path_, start_vec)) {
        LOG(INFO) << "The speed profile has been optimized...";
    } else {
        LOG(WARNING) << "Failed to optimize the speed profile...";
        return false;
    }

    // 3. generate the trajectory
    LOG(INFO) << "Start to generate the trajectory...";

    return true;
};

}   // namespace planning