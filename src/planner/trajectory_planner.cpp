#include "trajectory_planner.h"

namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param) {
    // Constructor
    backend_solver_ = solver_params.backend_solver().empty() ? "obca" : solver_params.backend_solver();
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);
    pwj_speed_ptr_     = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params(), solver_params.dt());
    admm_solver_ptr_   = std::make_unique<admmopt>();
    vehicle_model_ptr_ = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);
    initObcaSolver(solver_params, map_ptr);
};

void TrajPlanner::initObcaSolver(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr) {
    // Keep OBCA-specific setup out of the constructor body.
    std::string options;
    options += "Integer print_level  0\n";
    options += "String sb            yes\n";
    options += "Integer max_iter     10\n";
    options += "Numeric tol          1e-6\n";

    params::OBCAParams obca_params;
    if (solver_params.has_obca_params()) {
        obca_params = solver_params.obca_params();
    } else {
        obca_params.set_ref_weight(5.0);
        obca_params.set_smooth_weight(0.5);
        obca_params.set_control_weight(100.0);
    }

    obca_solver_ptr_ = std::make_unique<obcaopt>(options, vehicle_model_ptr_, map_ptr, obca_params);
}

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

    // 2. generate the trajectory with TrajOptimizer
    const std::vector<Eigen::Vector3d>* const frontend_path = hybrid_astar_ptr_->GetPath();
    LOG(INFO) << "Start to optimize the trajectory with backend solver: " << backend_solver_;
    if (!runBackendOpt(frontend_path, start_vec, goal_vec)) {
        LOG(WARNING) << "Failed to optimize the trajectory with backend solver: " << backend_solver_;
        return false;
    }

    return true;
};

bool TrajPlanner::runBackendOpt(const std::vector<Eigen::Vector3d>* const frontend_path,
                                const vehicle_model::VehiclePose&         start_vec,
                                const vehicle_model::VehiclePose&         goal_vec) {
    if (backend_solver_ == "obca" && obca_solver_ptr_ != nullptr) {
        auto sdv_path      = obcaopt::Vec3dToSdvPath(*frontend_path);
        auto start_vec_ptr = std::make_shared<vehicle_model::VehiclePose>(start_vec);
        auto goal_vec_ptr  = std::make_shared<vehicle_model::VehiclePose>(goal_vec);
        obca_solver_ptr_->Process(sdv_path, start_vec_ptr, goal_vec_ptr);
        setStatusControls(obca_solver_ptr_->GetStatesResult());
    } else if (backend_solver_ == "pwj" && pwj_speed_ptr_ != nullptr) {
        pwj_speed_ptr_->Optimize(*frontend_path, start_vec);
        setStatusControls(pwj_speed_ptr_->GetResult());
    } else {
        LOG(ERROR) << "The backend solver " << backend_solver_ << " is not supported or not initialized.";
        return false;
    }

    return true;
};

bool TrajPlanner::setStatusControls(const std::vector<Eigen::Vector4d>& opt_traj) {
    // 1. set the init status
    if (opt_traj.size() < 2) {
        LOG(WARNING) << "The opt traj size is less than 2";
        return false;
    }
    opt_status_.resize(opt_traj.size(), vehicle_model::KinematicModel::GetStateSize());
    opt_controls_.resize(opt_traj.size() - 1, vehicle_model::KinematicModel::GetControlSize());
    for (std::size_t i = 0; i < opt_traj.size(); ++i) {
        opt_status_(i, 0) = opt_traj[i].x();   // x
        opt_status_(i, 1) = opt_traj[i].y();   // y
        opt_status_(i, 2) = opt_traj[i].z();   // theta
        opt_status_(i, 3) = opt_traj[i].w();   // v
    }

    // 2. set the init controls using the forward difference
    const double dt = pwj_speed_ptr_->GetDt();
    const double L  = vehicle_model_ptr_->GetVehicleParam().length();
    for (std::size_t i = 0; i < opt_traj.size() - 1; ++i) {
        opt_controls_(i, 0) = (opt_traj[i + 1].w() - opt_traj[i].w()) / dt;   // a
        // compute steer angle
        double delta_theta = common::math::NormalizeAngle(opt_traj[i + 1].z() - opt_traj[i].z());
        double delta_x     = opt_traj[i + 1].x() - opt_traj[i].x();
        double delta_y     = opt_traj[i + 1].y() - opt_traj[i].y();
        double delta_s     = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (std::abs(delta_s) < kEpsilon) {
            opt_controls_(i, 1) = 0.0;
        } else {
            opt_controls_(i, 1) = std::atan2(L * delta_theta, delta_s);
            opt_controls_(i, 1) = common::math::NormalizeAngle(opt_controls_(i, 1));
        }
    }

    return true;
};

}   // namespace planning