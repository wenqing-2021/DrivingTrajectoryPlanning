#include "trajectory_planner.h"
#include <cmath>
#include <stdexcept>

namespace planning {

// Invert the rear-axle forward-Euler bicycle model used by RDA and OBCA.
// States: [x, y, heading, signed speed]; controls: [acceleration, steering].
Eigen::MatrixXd TrajPlanner::ReconstructTrajectoryControls(const Eigen::MatrixXd& states,
                                                     const std::vector<double>& timestamps,
                                                     double wheelbase) {
    if (states.cols() != 4 || states.rows() < 2 || !states.allFinite() ||
        timestamps.size() != static_cast<std::size_t>(states.rows()) ||
        !std::isfinite(wheelbase) || wheelbase <= 0.0) {
        throw std::invalid_argument("Invalid trajectory control reconstruction input");
    }
    Eigen::MatrixXd controls(states.rows() - 1, 2);
    double steering = 0.0;
    for (Eigen::Index i = 0; i < controls.rows(); ++i) {
        const double dt = timestamps[i + 1] - timestamps[i];
        if (!std::isfinite(timestamps[i]) || !std::isfinite(dt) || dt <= 0.0) {
            throw std::invalid_argument("Trajectory timestamps must be finite and increasing");
        }
        controls(i, 0) = (states(i + 1, 3) - states(i, 3)) / dt;
        const double heading_change = std::atan2(std::sin(states(i + 1, 2) - states(i, 2)),
                                                std::cos(states(i + 1, 2) - states(i, 2)));
        // At rest steering is unobservable from motion; ignore solver noise and
        // retain the previous angle. The initial steering is defined to be zero.
        if (i > 0 && std::abs(states(i, 3)) > 1e-4) {
            steering = std::atan(wheelbase * heading_change / (states(i, 3) * dt));
        }
        controls(i, 1) = steering;
    }
    return controls;
}


TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param,
                         std::shared_ptr<backend::RDAWorkspace> rda_workspace) {
    // initial common parameters
    backend_solver_    = solver_params.backend_solver().empty() ? "obca" : solver_params.backend_solver();
    vehicle_model_ptr_ = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);

    // initial frontend solver
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);

    // initial backend solvers
    pwj_speed_ptr_ = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params(), solver_params.dt());

    rda_solver_ptr_ =
        std::make_unique<rdaopt>(vehicle_model_ptr_, map_ptr, solver_params.rda_params(), solver_params.dt(),
                                 std::move(rda_workspace));
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
    } else if (backend_solver_ == "rda" && rda_solver_ptr_ != nullptr) {
        auto sdv_path      = rdaopt::Vec3dToSdvPath(*frontend_path);
        auto start_vec_ptr = std::make_shared<vehicle_model::VehiclePose>(start_vec);
        auto goal_vec_ptr  = std::make_shared<vehicle_model::VehiclePose>(goal_vec);
        if (!rda_solver_ptr_->Process(sdv_path, start_vec_ptr, goal_vec_ptr)) {
            LOG(WARNING) << "RDA solver failed to optimize the trajectory.";
            return false;
        }
        setstatesControls(opt_states_, opt_controls_, rda_solver_ptr_->GetStatesResult());
        setstatesControls(init_states_, init_controls_, rda_solver_ptr_->GetInitialStates());
    } else {
        LOG(ERROR) << "The backend solver " << backend_solver_ << " is not supported or not initialized.";
        return false;
    }

    // Reconstruct optimized controls using actual timing and signed velocity.
    const double nominal_dt = backend_solver_ == "obca" ? obca_solver_ptr_->GetDt() : pwj_speed_ptr_->GetDt();
    opt_timestamps_.assign(opt_states_.rows(), 0.0);
    for (std::size_t i = 1; i < opt_timestamps_.size(); ++i) {
        const double dt = backend_solver_ == "rda" ? rda_solver_ptr_->GetTimeSteps()(i - 1) : nominal_dt;
        opt_timestamps_[i] = opt_timestamps_[i - 1] + dt;
    }
    opt_controls_ = ReconstructTrajectoryControls(
        opt_states_, opt_timestamps_, vehicle_model_ptr_->GetVehicleParam().wheel_base());

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
    const double dt = backend_solver_ == "obca" ? obca_solver_ptr_->GetDt() : pwj_speed_ptr_->GetDt();
    const double L  = vehicle_model_ptr_->GetVehicleParam().wheel_base();
    for (std::size_t i = 0; i < opt_traj.size() - 1; ++i) {
        opt_controls(i, 0) = (opt_traj[i + 1].w() - opt_traj[i].w()) / dt;   // a
        // compute steer angle
        double delta_theta = common::math::NormalizeAngle(opt_traj[i + 1].z() - opt_traj[i].z());
        double delta_x     = opt_traj[i + 1].x() - opt_traj[i].x();
        double delta_y     = opt_traj[i + 1].y() - opt_traj[i].y();
        double delta_s     = delta_x * std::cos(opt_traj[i].z()) + delta_y * std::sin(opt_traj[i].z());
        if (i == 0 || std::abs(delta_s) < kEpsilon) {
            opt_controls(i, 1) = 0.0;
        } else {
            opt_controls(i, 1) = std::atan(L * delta_theta / delta_s);
        }
    }

    return true;
};

}   // namespace planning
