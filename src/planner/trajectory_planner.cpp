#include "trajectory_planner.h"

namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param) {
    // initial common parameters
    backend_solver_    = solver_params.backend_solver().empty() ? "obca" : solver_params.backend_solver();
    vehicle_model_ptr_ = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);

    // initial frontend solver
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);

    // initial backend solvers
    pwj_speed_ptr_ = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params(), solver_params.dt());

    rda_solver_ptr_ =
        std::make_unique<rdaopt>(vehicle_model_ptr_, map_ptr, solver_params.rda_params(), solver_params.dt());
    initObcaSolver(solver_params, map_ptr);
};

void TrajPlanner::initObcaSolver(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr) {
    params::OBCAParams obca_params;
    if (solver_params.has_obca_params()) {
        obca_params = solver_params.obca_params();
    } else {
        obca_params.set_ref_weight(5.0);
        obca_params.set_smooth_weight(0.5);
        obca_params.set_control_weight(100.0);
        obca_params.set_ipopt_max_iter(30);
        obca_params.set_ipopt_tol(1e-3);
        obca_params.set_ipopt_acceptable_tol(5e-2);
        obca_params.set_ipopt_acceptable_iter(10);
        obca_params.set_linear_solver("mumps");
    }

    const int32_t ipopt_max_iter = obca_params.ipopt_max_iter() > 0 ? obca_params.ipopt_max_iter() : 30;
    const double  ipopt_tol      = obca_params.ipopt_tol() > 0.0 ? obca_params.ipopt_tol() : 1e-3;
    const double  ipopt_acceptable_tol =
        obca_params.ipopt_acceptable_tol() > 0.0 ? obca_params.ipopt_acceptable_tol() : 5e-2;
    const int32_t ipopt_acceptable_iter =
        obca_params.ipopt_acceptable_iter() > 0 ? obca_params.ipopt_acceptable_iter() : 10;
    const std::string ipopt_linear_solver = obca_params.linear_solver().empty() ? "mumps" : obca_params.linear_solver();

    // Keep OBCA-specific setup out of the constructor body.
    std::string options;
    options += "Integer print_level      5\n";
    options += "String sb                yes\n";
    options += "Integer max_iter         " + std::to_string(ipopt_max_iter) + "\n";
    options += "Numeric tol              " + std::to_string(ipopt_tol) + "\n";
    options += "Numeric acceptable_tol   " + std::to_string(ipopt_acceptable_tol) + "\n";
    options += "Integer acceptable_iter  " + std::to_string(ipopt_acceptable_iter) + "\n";
    options += "String linear_solver     " + ipopt_linear_solver + "\n";
    options += "String mu_strategy       adaptive\n";

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
        setstatesControls(opt_states_, opt_controls_, obca_solver_ptr_->GetStatesResult());
        setstatesControls(init_states_, init_controls_, obca_solver_ptr_->GetInitStates());
    } else if (backend_solver_ == "pwj" && pwj_speed_ptr_ != nullptr) {
        pwj_speed_ptr_->Optimize(*frontend_path, start_vec);
        setstatesControls(opt_states_, opt_controls_, pwj_speed_ptr_->GetResult());
    } else {
        LOG(ERROR) << "The backend solver " << backend_solver_ << " is not supported or not initialized.";
        return false;
    }

    return true;
};

bool TrajPlanner::setstatesControls(vehicle_model::opt_states& opt_states, vehicle_model::opt_control& opt_controls,
                                    const std::vector<Eigen::Vector4d>& opt_traj) {
    // 1. set the init states
    if (opt_traj.size() < 2) {
        LOG(WARNING) << "The opt traj size is less than 2";
        return false;
    }
    opt_states.resize(opt_traj.size(), vehicle_model::KinematicModel::GetStateSize());
    opt_controls.resize(opt_traj.size() - 1, vehicle_model::KinematicModel::GetControlSize());
    for (std::size_t i = 0; i < opt_traj.size(); ++i) {
        opt_states(i, 0) = opt_traj[i].x();   // x
        opt_states(i, 1) = opt_traj[i].y();   // y
        opt_states(i, 2) = opt_traj[i].z();   // theta
        opt_states(i, 3) = opt_traj[i].w();   // v
    }

    // 2. set the init controls using the forward difference
    const double dt = pwj_speed_ptr_->GetDt();
    const double L  = vehicle_model_ptr_->GetVehicleParam().length();
    for (std::size_t i = 0; i < opt_traj.size() - 1; ++i) {
        opt_controls(i, 0) = (opt_traj[i + 1].w() - opt_traj[i].w()) / dt;   // a
        // compute steer angle
        double delta_theta = common::math::NormalizeAngle(opt_traj[i + 1].z() - opt_traj[i].z());
        double delta_x     = opt_traj[i + 1].x() - opt_traj[i].x();
        double delta_y     = opt_traj[i + 1].y() - opt_traj[i].y();
        double delta_s     = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (std::abs(delta_s) < kEpsilon) {
            opt_controls(i, 1) = 0.0;
        } else {
            opt_controls(i, 1) = std::atan2(L * delta_theta, delta_s);
            opt_controls(i, 1) = common::math::NormalizeAngle(opt_controls(i, 1));
        }
    }

    return true;
};

}   // namespace planning