#include "trajectory_planner.h"
#include <chrono>

namespace planning {

TrajPlanner::TrajPlanner(const params::SolverParams& solver_params, const std::shared_ptr<map::Map>& map_ptr,
                         const std::shared_ptr<collision_check::BaseCheck>& collision_checker,
                         const kinematic_model::VehicleParam&               vehicle_param,
                         std::shared_ptr<backend::RDAWorkspace> rda_workspace,
                         std::shared_ptr<backend::OCEANWorkspace> ocean_workspace) {
    // initial common parameters
    backend_solver_    = solver_params.backend_solver().empty() ? "obca" : solver_params.backend_solver();
    vehicle_model_ptr_ = std::make_shared<vehicle_model::KinematicModel>(vehicle_param);

    nominal_dt_ = solver_params.dt();
    diagnostics_.set_backend(backend_solver_);

    // initial frontend solver
    hybrid_astar_ptr_ =
        std::make_unique<frontend::HybridAstar>(solver_params.hybrid_a_star_param(), map_ptr, collision_checker);

    // Construct only the selected backend.
    if (backend_solver_ == "pwj")
        pwj_speed_ptr_ = std::make_unique<pwjspeed>(solver_params.piesewise_jerk_params(), solver_params.dt());
    else if (backend_solver_ == "rda")
        rda_solver_ptr_ = std::make_unique<rdaopt>(vehicle_model_ptr_, map_ptr, solver_params.rda_params(),
                                                  solver_params.dt(), std::move(rda_workspace));
    else if (backend_solver_ == "ocean")
        ocean_solver_ptr_ =
            std::make_unique<backend::OCEANSolver>(vehicle_model_ptr_, map_ptr, solver_params.ocean_params(),
                                                   solver_params.dt(), std::move(ocean_workspace));
    else if (backend_solver_ == "obca") initObcaSolver(solver_params, map_ptr);
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
    const auto frontend_start = std::chrono::steady_clock::now();
    // 1. frontend path plan
    LOG(INFO) << "Start to search the frontend path...";
    if (hybrid_astar_ptr_->Plan(start_vec, goal_vec)) {
        LOG(INFO) << "The frontend path is found...";
        hybrid_astar_ptr_->GetPathLength(frontend_path_length_);
    } else {
        LOG(WARNING) << "Failed to find the frontend path...";
        debug_node_list_ = hybrid_astar_ptr_->GetDebugNodeList();
        LOG(INFO) << "The debug node list size is: " << debug_node_list_.size();
        diagnostics_.set_status("frontend_failed");
        return false;
    }

    diagnostics_.set_frontend_ms(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frontend_start).count());
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
    // OCEAN owns its control and time-interval variables, so it also owns the
    // timestamps and returns both directly, without the shared timing block below.
    if (backend_solver_ == "ocean" && ocean_solver_ptr_) {
        vehicle_model::sdv_path path;
        for (const auto& point : *frontend_path) path.push_back({point.x(), point.y(), point.z()});
        auto         start       = std::make_shared<vehicle_model::VehiclePose>(start_vec);
        auto         goal        = std::make_shared<vehicle_model::VehiclePose>(goal_vec);
        const double frontend_ms = diagnostics_.frontend_ms();
        ocean_solver_ptr_->SetTimeBudget(remaining_budget_ - frontend_ms / 1000.0);
        const bool success = ocean_solver_ptr_->Process(path, start, goal);
        diagnostics_       = ocean_solver_ptr_->GetStats();
        diagnostics_.set_frontend_ms(frontend_ms);
        if (!success) return false;
        if (!setStates(opt_states_, ocean_solver_ptr_->GetStatesResult()) ||
            !setStates(init_states_, ocean_solver_ptr_->GetInitialStates()) ||
            !setControls(opt_controls_, ocean_solver_ptr_->GetControlsResult(), opt_states_.rows() - 1))
            return false;
        const Eigen::VectorXd& intervals = ocean_solver_ptr_->GetTimeSteps();
        opt_timestamps_.assign(opt_states_.rows(), 0.0);
        for (Eigen::Index k = 0; k < intervals.size(); ++k) opt_timestamps_[k + 1] = opt_timestamps_[k] + intervals[k];
        return true;
    }
    if (backend_solver_ == "obca" && obca_solver_ptr_ != nullptr) {
        auto sdv_path      = obcaopt::Vec3dToSdvPath(*frontend_path);
        auto start_vec_ptr = std::make_shared<vehicle_model::VehiclePose>(start_vec);
        auto goal_vec_ptr  = std::make_shared<vehicle_model::VehiclePose>(goal_vec);
        obca_solver_ptr_->Process(sdv_path, start_vec_ptr, goal_vec_ptr);
        const auto&     obca_controls = obca_solver_ptr_->GetControlsResult();
        Eigen::MatrixXd native(obca_controls.size(), 2);
        for (std::size_t i = 0; i < obca_controls.size(); ++i) native.row(i) = obca_controls[i];
        diagnostics_.set_control_source("native");
        if (!setStates(opt_states_, obca_solver_ptr_->GetStatesResult()) ||
            !setStates(init_states_, obca_solver_ptr_->GetInitStates()) ||
            !setControls(opt_controls_, native, opt_states_.rows() - 1))
            return false;
    } else if (backend_solver_ == "pwj" && pwj_speed_ptr_ != nullptr) {
        pwj_speed_ptr_->Optimize(*frontend_path, start_vec);
        if (!setStates(opt_states_, pwj_speed_ptr_->GetResult())) return false;
        // The piecewise-jerk planner optimizes a speed profile along the path, so it
        // has no commands to export; an empty control list is a valid result.
        opt_controls_.resize(0, 0);
        diagnostics_.set_control_source("none");
    } else if (backend_solver_ == "rda" && rda_solver_ptr_ != nullptr) {
        auto sdv_path      = rdaopt::Vec3dToSdvPath(*frontend_path);
        auto start_vec_ptr = std::make_shared<vehicle_model::VehiclePose>(start_vec);
        auto goal_vec_ptr  = std::make_shared<vehicle_model::VehiclePose>(goal_vec);
        if (!rda_solver_ptr_->Process(sdv_path, start_vec_ptr, goal_vec_ptr)) {
            LOG(WARNING) << "RDA solver failed to optimize the trajectory.";
            diagnostics_.set_status("backend_failed");
            return false;
        }
        const auto& timings = rda_solver_ptr_->GetTimings();
        diagnostics_.set_iterations(timings.iterations);
        diagnostics_.set_prepare_ms(timings.prepare_ms);
        diagnostics_.set_su_assembly_ms(timings.su_assembly_ms);
        diagnostics_.set_su_solve_ms(timings.su_solve_ms);
        diagnostics_.set_cone_assembly_ms(timings.cone_assembly_ms);
        diagnostics_.set_cone_solve_ms(timings.cone_solve_ms);
        diagnostics_.set_control_source("native");
        if (!setStates(opt_states_, rda_solver_ptr_->GetStatesResult()) ||
            !setStates(init_states_, rda_solver_ptr_->GetInitialStates()) ||
            !setControls(opt_controls_, rda_solver_ptr_->GetControlsResult(), opt_states_.rows() - 1))
            return false;
    } else {
        LOG(ERROR) << "The backend solver " << backend_solver_ << " is not supported or not initialized.";
        diagnostics_.set_status("unsupported_backend");
        return false;
    }

    // Timing is each backend's own: RDA accumulates its optimized per-segment
    // interval, OBCA and the speed planner use their configured solver interval.
    const double nominal_dt = backend_solver_ == "obca" ? obca_solver_ptr_->GetDt() : nominal_dt_;
    opt_timestamps_.assign(opt_states_.rows(), 0.0);
    for (std::size_t i = 1; i < opt_timestamps_.size(); ++i) {
        const double dt = backend_solver_ == "rda" ? rda_solver_ptr_->GetTimeSteps()(i - 1) : nominal_dt;
        opt_timestamps_[i] = opt_timestamps_[i - 1] + dt;
    }
    diagnostics_.set_status("success");
    diagnostics_.set_segments(static_cast<int>(opt_states_.rows()) - 1);

    return true;
};

bool TrajPlanner::setStates(vehicle_model::opt_states& states, const std::vector<Eigen::Vector4d>& traj) {
    if (traj.size() < 2) {
        LOG(WARNING) << "The opt traj size is less than 2";
        return false;
    }
    states.resize(traj.size(), vehicle_model::KinematicModel::GetStateSize());
    for (std::size_t i = 0; i < traj.size(); ++i) {
        states(i, 0) = traj[i].x();   // x
        states(i, 1) = traj[i].y();   // y
        states(i, 2) = traj[i].z();   // theta
        states(i, 3) = traj[i].w();   // v
    }
    return true;
};

bool TrajPlanner::setControls(vehicle_model::opt_control& controls, const Eigen::MatrixXd& native,
                              Eigen::Index segments) {
    if (segments < 0 || native.rows() != segments || native.cols() != 2 || !native.allFinite()) {
        LOG(ERROR) << "The backend returned " << native.rows() << " control rows for " << segments
                   << " segments";
        return false;
    }
    controls = native;
    return true;
};

}   // namespace planning
