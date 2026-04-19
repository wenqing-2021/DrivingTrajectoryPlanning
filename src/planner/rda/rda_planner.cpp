#include "rda_planner.h"

#include <cmath>

namespace planning {
namespace backend {

// ─── Constructor ────────────────────────────────────────────────────────────

RDASolver::RDASolver(const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
                     const std::shared_ptr<map::Map>& map_ptr, const params::RDAParams& rda_params, double dt)
    : dynamic_model_ptr_(dynamic_model_ptr)
    , map_ptr_(map_ptr)
    , rda_params_(rda_params)
    , dt_(dt)
    , N_(0)
    , state_dim_(vehicle_model::KinematicModel::GetStateSize())
    , control_dim_(vehicle_model::KinematicModel::GetControlSize())
    , total_var_dim_(0)
    , convergence_tolerance_(1e-4)
    , max_rda_iterations_(10)
    , penalty_weight_(1.0)
    , use_warm_start_(false) {
    if (!dynamic_model_ptr_ || !map_ptr_) {
        throw std::runtime_error("RDASolver: dynamic_model_ptr or map_ptr is null.");
    }
    const auto& vp      = dynamic_model_ptr_->GetVehicleParam();
    wheel_base_         = vp.wheel_base();
    max_velocity_       = vp.max_velocity();
    max_acceleration_   = vp.max_acc();
    max_steering_angle_ = vp.max_steer_angle();
    vehicle_length_     = vp.length();
    vehicle_width_      = vp.width();
}

// ─── Main interface ──────────────────────────────────────────────────────────

bool RDASolver::Process(const vehicle_model::sdv_path&               init_path,
                        std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                        std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    return false;   // TODO: implement
}

// ─── Result accessors ────────────────────────────────────────────────────────

const RDAOptimizationResult& RDASolver::GetOptimizationResult() const {
    return optimization_result_;
}

const std::vector<Eigen::Vector4d>& RDASolver::GetStatesResult() const {
    return states_result_;
}

const std::vector<Eigen::Vector2d>& RDASolver::GetControlsResult() const {
    return controls_result_;
}

const std::vector<Eigen::Vector4d>& RDASolver::GetInitialStates() const {
    return initial_states_;
}

double RDASolver::GetOptimalCost() const {
    return optimization_result_.cost;
}

// ─── Utility ─────────────────────────────────────────────────────────────────

vehicle_model::sdv_path RDASolver::Vec3dToSdvPath(const std::vector<Eigen::Vector3d>& path) {
    vehicle_model::sdv_path sdv_path;
    for (const auto& p : path) {
        vehicle_model::VehiclePose pose;
        pose.x     = p.x();
        pose.y     = p.y();
        pose.theta = p.z();
        sdv_path.push_back(pose);
    }
    return sdv_path;
}

std::vector<Eigen::Vector3d> RDASolver::SdvPathToVec3d(const vehicle_model::sdv_path& path) {
    std::vector<Eigen::Vector3d> result;
    result.reserve(path.size());
    for (const auto& pose : path) { result.emplace_back(pose.x, pose.y, pose.theta); }
    return result;
}

// ─── Private: problem formulation ────────────────────────────────────────────

bool RDASolver::formSOCPProblem(const vehicle_model::sdv_path&               init_path,
                                std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    return false;   // TODO: implement
}

bool RDASolver::setInitVariable(const vehicle_model::sdv_path&               init_path,
                                std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    if (init_path.size() < 2) {
        LOG(WARNING) << "RDASolver::setInitVariable failed: init_path size is less than 2.";
        return false;
    }
    if (!start_pose_ptr || !goal_pose_ptr) {
        LOG(WARNING) << "RDASolver::setInitVariable failed: start_pose_ptr or goal_pose_ptr is null.";
        return false;
    }

    N_             = init_path.size();
    total_var_dim_ = N_ * (state_dim_ + control_dim_);
    initial_variables_.setZero(total_var_dim_);
    initial_states_.clear();
    initial_states_.reserve(N_);

    // Initialize state and control variables
    if (!initializeStateControl(init_path)) {
        LOG(WARNING) << "RDASolver::setInitVariable failed: initializeStateControl returned false.";
        return false;
    }

    return true;
}

bool RDASolver::initializeStateControl(const vehicle_model::sdv_path& init_path) {
    auto clamp_value = [](double val, double lower, double upper) { return std::max(lower, std::min(val, upper)); };
    auto idx         = [](VariableIdx index) { return static_cast<std::size_t>(index); };

    // First pass: estimate signed velocity from geometric progression of waypoints.
    std::vector<double> v_estimates(N_, 0.0);
    for (std::size_t i = 1; i + 1 < N_; ++i) {
        const double dx   = init_path[i + 1].x - init_path[i].x;
        const double dy   = init_path[i + 1].y - init_path[i].y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double sign = (dx * std::cos(init_path[i].theta) + dy * std::sin(init_path[i].theta)) >= 0.0 ? 1.0 : -1.0;
        v_estimates[i]    = clamp_value(sign * dist / dt_, -max_velocity_, max_velocity_);
    }
    v_estimates.front() = 0.0;
    v_estimates.back()  = 0.0;

    // Gear transition handling: force zero velocity at direction switch points.
    for (std::size_t i = 1; i + 1 < N_; ++i) {
        if (v_estimates[i - 1] * v_estimates[i + 1] < 0.0) { v_estimates[i] = 0.0; }
    }

    constexpr double kPi = 3.14159265358979323846;

    // Second pass: set state and control initialization.
    for (std::size_t i = 0; i < N_; ++i) {
        const std::size_t base                             = i * (state_dim_ + control_dim_);
        initial_variables_[base + idx(VariableIdx::X)]     = init_path[i].x;
        initial_variables_[base + idx(VariableIdx::Y)]     = init_path[i].y;
        initial_variables_[base + idx(VariableIdx::THETA)] = init_path[i].theta;
        initial_variables_[base + idx(VariableIdx::V)]     = v_estimates[i];

        initial_states_.emplace_back(init_path[i].x, init_path[i].y, init_path[i].theta, v_estimates[i]);

        double steer_init = 0.0;
        if (i + 1 < N_ && std::abs(v_estimates[i]) > 1e-3) {
            double d_theta = init_path[i + 1].theta - init_path[i].theta;
            while (d_theta > kPi) d_theta -= 2.0 * kPi;
            while (d_theta < -kPi) d_theta += 2.0 * kPi;
            steer_init = std::atan(wheel_base_ * d_theta / (v_estimates[i] * dt_));
            steer_init = clamp_value(steer_init, -max_steering_angle_, max_steering_angle_);
        }

        double acc_init = 0.0;
        if (i + 1 < N_) {
            acc_init = (v_estimates[i + 1] - v_estimates[i]) / dt_;
            acc_init = clamp_value(acc_init, -max_acceleration_, max_acceleration_);
        }

        initial_variables_[base + idx(VariableIdx::ACCELERATION)] = acc_init;
        initial_variables_[base + idx(VariableIdx::STEER_ANGLE)]  = steer_init;
    }

    return true;
}

bool RDASolver::initializeDualVariables() {
    // 1. get the obstacle from map
    if (map_ptr_ == nullptr) {
        LOG(WARNING) << "The map pointer is null.";
        return false;
    }
    const auto& obs_list         = map_ptr_->GetObsList();
    std::size_t obstacle_num     = obs_list.size();
    std::size_t obstacle_num_pts = 0;
    for (const auto& obs : obs_list) { obstacle_num_pts += obs.num_points(); }

    // 2. Initialize dual variables
    std::size_t lambda_num = obstacle_num_pts * N_;
    std::size_t mu_num     = obstacle_num * N_ * kVehicleBoundaryNum;
    std::size_t z_num      = N_ * obstacle_num;
    dual_variables_.setOnes(lambda_num + mu_num + z_num);
    dual_variables_ *= 0.1;   // Small initial values to help convergence

    return true;
}

// ─── Private: constraints ────────────────────────────────────────────────────

bool RDASolver::setVariableBounds(Eigen::VectorXd& x_lower, Eigen::VectorXd& x_upper) {
    return false;   // TODO: implement
}

bool RDASolver::setDynamicConstraints(Eigen::MatrixXd& A_constraint, Eigen::VectorXd& b_constraint) {
    return false;   // TODO: implement
}

bool RDASolver::setCollisionAvoidanceConstraints(Eigen::MatrixXd&              A_collision,
                                                 std::vector<Eigen::VectorXd>& b_collision_vec) {
    return false;   // TODO: implement
}

// ─── Private: SOCP solver ────────────────────────────────────────────────────

bool RDASolver::solveSOCPProblem(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& A,
                                 const Eigen::VectorXd& b, Eigen::VectorXd& solution) {
    return false;   // TODO: implement
}

// ─── Private: RDA iteration ──────────────────────────────────────────────────

// ─── Private: obstacle / cost helpers ────────────────────────────────────────

bool RDASolver::setupQuadraticCostFunction(Eigen::MatrixXd& P, Eigen::VectorXd& q) {
    return false;   // TODO: implement
}

}   // namespace backend
}   // namespace planning
