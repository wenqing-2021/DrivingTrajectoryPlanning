#include "obca_planner.h"

namespace planning {
namespace backend {

bool OBCASolver::Process(const vehicle_model::sdv_path&               init_path,
                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    if (init_path.size() < 2) {
        LOG(WARNING) << "The initial path is not valid, the size is less than 2.";
        return false;
    }
    if (start_pose_ptr == nullptr || goal_pose_ptr == nullptr) {
        LOG(WARNING) << "The start pose or goal pose pointer is null.";
        return false;
    }
    Dvector init_variables;
    // 1. get the initial variables
    if (!setInitVariable(init_path, &fg_eval_, &init_variables, start_pose_ptr, goal_pose_ptr)) {
        LOG(WARNING) << "Failed to set the initial variables.";
        return false;
    }

    // 2. get the hyperlane
    if (!fg_eval_.getObstacleBound(map_ptr_)) {
        LOG(WARNING) << "Failed to get the obstacle boundary.";
        return false;
    }

    // 3. set the constraints
    Dvector xl, xu, gl, gu;
    fg_eval_.setConstraintsBound(&xl, &xu, &gl, &gu);

    // 4.check the variable num is aligned with the constraint num
    if (init_variables.size() != xl.size() || init_variables.size() != xu.size()) {
        LOG(WARNING) << "The variable size is not aligned with the constraint size.";
        return false;
    }
    // update variable nums and constraint nums
    fg_eval_.updateProblemSize(init_variables.size(), gl.size());
    CppAD::ipopt::solve_result<Dvector> solution;   // solution
    CppAD::ipopt::solve<Dvector, FG_eval>(
        this->getOptions(), init_variables, xl, xu, gl, gu, fg_eval_, solution);   // solve the problem

    // 5. get the result
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
        LOG(WARNING) << "The solver failed to find a solution.";
        LOG(WARNING) << "Solver status: " << solution.status;
        // return false;
    }

    // 6. store the result
    if (!setResult(solution)) {
        LOG(WARNING) << "Failed to set the result.";
        return false;
    }

    return true;
};

bool OBCASolver::setResult(const CppAD::ipopt::solve_result<Dvector>& solution) {
    states_result_.clear();
    controls_result_.clear();

    std::size_t N = N_;
    for (std::size_t i = 0; i < N; ++i) {
        Eigen::Vector4d state;
        state(0) = solution.x[i * (state_num_ + control_num_) + VariableIndex::X];
        state(1) = solution.x[i * (state_num_ + control_num_) + VariableIndex::Y];
        state(2) = solution.x[i * (state_num_ + control_num_) + VariableIndex::THETA];
        state(3) = solution.x[i * (state_num_ + control_num_) + VariableIndex::V];
        states_result_.emplace_back(state);
        if (i < N - 1) {
            Eigen::Vector2d control;
            control(0) = solution.x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
            control(1) = solution.x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
            controls_result_.emplace_back(control);
        }
    }


    return true;
}

bool OBCASolver::setInitVariable(const vehicle_model::sdv_path& init_path, OBCAFG_eval* fg_eval,
                                 Dvector* init_variables, std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                 std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    // Initialize control states first
    if (!initializeControlStates(init_path, start_pose_ptr, goal_pose_ptr)) {
        LOG(WARNING) << "Failed to initialize control states.";
        return false;
    }

    // Then initialize dual variables
    if (!initializeDualVariables(fg_eval, init_variables, start_pose_ptr, goal_pose_ptr)) {
        LOG(WARNING) << "Failed to initialize dual variables.";
        return false;
    }

    return true;
};

bool OBCASolver::initializeControlStates(const vehicle_model::sdv_path&               init_path,
                                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    // 1. Set the initial variables for the optimization problem.
    // NOTE: initial path including the start pose and the goal pose
    N_ = init_path.size();
    x0_.resize(N_ * (state_num_ + control_num_));
    const auto   vehicle_param = dynamic_model_ptr_->GetVehicleParam();
    const double wheel_base    = vehicle_param.wheel_base();
    const double max_v         = vehicle_param.max_velocity();
    const double max_acc       = vehicle_param.max_acc();
    const double max_steer     = vehicle_param.max_steer_angle();

    auto clamp_value = [](double val, double lower, double upper) { return std::max(lower, std::min(val, upper)); };

    // First pass: estimate velocities accounting for gear changes (forward/reverse transitions).
    // Velocity is a vector here: positive=forward, negative=reverse.
    std::vector<double> v_estimates(N_, 0.0);
    for (std::size_t i = 1; i < N_ - 1; ++i) {
        double dx   = init_path[i + 1].x - init_path[i].x;
        double dy   = init_path[i + 1].y - init_path[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);

        // Velocity direction: positive if motion aligns with heading, negative if opposite.
        double sign    = (dx * std::cos(init_path[i].theta) + dy * std::sin(init_path[i].theta)) >= 0.0 ? 1.0 : -1.0;
        v_estimates[i] = clamp_value(sign * dist / kDt, -max_v, max_v);
    }
    // Start and end velocities are fixed to 0.
    v_estimates[0]      = 0.0;
    v_estimates[N_ - 1] = 0.0;

    // Detect gear change points: where velocity changes sign.
    for (std::size_t i = 1; i < N_ - 1; ++i) {
        if (v_estimates[i - 1] * v_estimates[i + 1] < 0.0) {
            // Sign change detected between neighbors: this is a gear transition.
            v_estimates[i] = 0.0;
        }
    }

    // Second pass: set all variables using estimated velocities and accelerations
    initial_states_.clear();
    for (std::size_t i = 0; i < N_; ++i) {
        x0_[i * (state_num_ + control_num_) + VariableIndex::X]     = init_path[i].x;
        x0_[i * (state_num_ + control_num_) + VariableIndex::Y]     = init_path[i].y;
        x0_[i * (state_num_ + control_num_) + VariableIndex::THETA] = init_path[i].theta;
        x0_[i * (state_num_ + control_num_) + VariableIndex::V]     = v_estimates[i];

        // set the initial states
        initial_states_.emplace_back(
            Eigen::Vector4d(init_path[i].x, init_path[i].y, init_path[i].theta, v_estimates[i]));

        // Estimate initial steering angle from heading change: steer = atan(L * dθ/ds).
        double steer_init = 0.0;
        if (i < N_ - 1 && std::abs(v_estimates[i]) > 1e-3) {
            double d_theta = init_path[i + 1].theta - init_path[i].theta;
            // Normalize angle difference to [-pi, pi] to handle wrap-around.
            while (d_theta > M_PI) d_theta -= 2.0 * M_PI;
            while (d_theta < -M_PI) d_theta += 2.0 * M_PI;
            steer_init = std::atan(wheel_base * d_theta / (v_estimates[i] * kDt));
            steer_init = clamp_value(steer_init, -max_steer, max_steer);
        }
        x0_[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE] = steer_init;

        // Estimate initial acceleration from velocity change: a = (v_{i+1} - v_i) / dt
        // Start and end accelerations are fixed to 0
        double acc_init = 0.0;
        if (i < N_ - 1) {
            acc_init = (v_estimates[i + 1] - v_estimates[i]) / kDt;
            acc_init = clamp_value(acc_init, -max_acc, max_acc);
        }
        x0_[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION] = acc_init;
    }

    return true;
}

bool OBCASolver::initializeDualVariables(OBCAFG_eval* fg_eval, Dvector* init_variables,
                                         std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                                         std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
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
    std::size_t     lambda_num = obstacle_num_pts * N_;
    std::size_t     mu_num     = obstacle_num * N_ * kVehicleBoundaryNum;
    Eigen::VectorXd lambda0_(lambda_num);
    Eigen::VectorXd mu0_(mu_num);
    mu0_.setOnes();
    mu0_ *= 0.1;
    lambda0_.setOnes();
    lambda0_ *= 0.1;

    // 3. Append dual variables to primal variables
    x0_.conservativeResize(x0_.size() + lambda0_.size() + mu0_.size());
    std::size_t offset                                 = x0_.size() - lambda0_.size() - mu0_.size();
    x0_.segment(offset, lambda0_.size())               = lambda0_;   // [offset, offset+lambda_size)
    x0_.segment(offset + lambda0_.size(), mu0_.size()) = mu0_;       // [offset+lambda_size, end)

    // 4. Set init parameters in fg_eval and copy to init_variables
    auto x0_ptr = std::make_shared<Eigen::VectorXd>(x0_);
    fg_eval->setInitParameters(
        x0_ptr, N_, state_num_, control_num_, lambda_num, mu_num, dynamic_model_ptr_, start_pose_ptr, goal_pose_ptr);
    for (int i = 0; i < x0_.size(); ++i) { init_variables->push_back(x0_[i]); }

    return true;
};

bool OBCAFG_eval::getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                               Eigen::VectorXd& obstacle_b, std::size_t i) {
    // 1. compute the hyperlane for obstacle
    obstacle_A.resize(obstacle.num_points(), 2);
    obstacle_b.resize(obstacle.num_points());

    // 2.1 check if the obstacle is valid
    const int obstacle_num_pts = obstacle.num_points();
    if (obstacle_num_pts < 3) {
        LOG(WARNING) << "The obstacle_" << i << "is not valid, the size is less than 3.";
        return false;
    }
    const auto line_segments = obstacle.line_segments();
    const auto center_pts    = obstacle.center();
    for (std::size_t j = 0; j < obstacle_num_pts; ++j) {
        LOG(INFO) << "Processing obstacle_" << i << ", edge_" << j << ": start(" << line_segments[j].start().x() << ", "
                  << line_segments[j].start().y() << "), end(" << line_segments[j].end().x() << ", "
                  << line_segments[j].end().y() << ")";
        const auto edge_vector = line_segments[j].unit_direction();
        // get the direction of the vector
        common::math::Vec2d normal_vector(-edge_vector.y(), edge_vector.x());
        common::math::Vec2d vec_to_center(center_pts.x() - line_segments[j].start().x(),
                                          center_pts.y() - line_segments[j].start().y());
        if (normal_vector.InnerProd(vec_to_center) > 0) { normal_vector *= -1; }
        obstacle_A(j, 0) = normal_vector.x();
        obstacle_A(j, 1) = normal_vector.y();
        obstacle_b(j)    = normal_vector.InnerProd(line_segments[j].start());
    }
    LOG(INFO) << "obstacle_" << i << " hyperlane is generated successfully.";

    return true;
}

bool OBCAFG_eval::getRotationMatrix(const CppAD::AD<double> theta, CppMatrixXd& R) {
    R.resize(2, 2);
    R(0, 0) = CppAD::cos(theta);
    R(0, 1) = -CppAD::sin(theta);
    R(1, 0) = CppAD::sin(theta);
    R(1, 1) = CppAD::cos(theta);

    return true;
}

bool OBCAFG_eval::getMovementMatrix(const CppAD::AD<double> x, const CppAD::AD<double> y, const CppAD::AD<double> theta,
                                    const double off_set, CppVecXd& t) {
    // get the movement matrix
    t.resize(2, 1);
    t(0, 0) = x + off_set * CppAD::cos(theta);
    t(1, 0) = y + off_set * CppAD::sin(theta);

    return true;
}

CppAD::AD<double> OBCAFG_eval::getCostFunction(const ADvector& x) {
    CppAD::AD<double> cost           = 0.0;
    double            ref_weight     = obca_params_.ref_weight();
    double            control_weight = obca_params_.control_weight();
    for (std::size_t i = 0; i < N_ - 1; ++i) {
        // Match Python OBCA objective: state tracking + control effort, no control smoothness term.
        CppAD::AD<double> x_pos = x[i * (state_num_ + control_num_) + VariableIndex::X];
        CppAD::AD<double> y_pos = x[i * (state_num_ + control_num_) + VariableIndex::Y];
        CppAD::AD<double> theta = x[i * (state_num_ + control_num_) + VariableIndex::THETA];

        cost += ref_weight * (x_pos - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::X]) *
                (x_pos - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::X]);
        cost += ref_weight * (y_pos - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::Y]) *
                (y_pos - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::Y]);
        cost += ref_weight * (theta - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::THETA]) *
                (theta - (*ref_X_ptr_)[i * (state_num_ + control_num_) + VariableIndex::THETA]);

        cost += control_weight * x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE] *
                x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
        cost += control_weight * x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION] *
                x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
    }

    return cost;
}

bool OBCAFG_eval::setConstraintsBound(IpoptSolver::Dvector* xl, IpoptSolver::Dvector* xu, IpoptSolver::Dvector* gl,
                                      IpoptSolver::Dvector* gu) {
    auto _build_bound = [](IpoptSolver::Dvector* target, const IpoptSolver::Dvector* const source) {
        std::size_t size = source->size();
        target->resize(size);
        for (std::size_t i = 0; i < size; ++i) { (*target)[i] = (*source)[i]; }
    };
    // 1. set pose constraint
    pose_constraints_num_ = 8;
    IpoptSolver::Dvector pose_gl, pose_gu;
    setPoseConstraintsBound(&pose_gl, &pose_gu);

    // 2. set dynamic constraint
    IpoptSolver::Dvector dynamic_gl, dynamic_gu;
    setDynamicConstraintsBound(&dynamic_gl, &dynamic_gu);

    // 3. control feasibility constraint
    IpoptSolver::Dvector control_xl, control_xu;
    setControlFeasibleConstraintsBound(&control_xl, &control_xu);

    // 4. avoidance constraint
    IpoptSolver::Dvector avoidanc_gl, avoidanc_gu;
    setAvoidanceConstraintsBound(&avoidanc_gl, &avoidanc_gu);

    // 5. build the bound
    pose_gl.push_vector(dynamic_gl);
    pose_gl.push_vector(avoidanc_gl);
    pose_gu.push_vector(dynamic_gu);
    pose_gu.push_vector(avoidanc_gu);

    _build_bound(xl, &control_xl);
    _build_bound(xu, &control_xu);
    _build_bound(gl, &pose_gl);
    _build_bound(gu, &pose_gu);

    return true;
}

FG_eval::ADvector OBCAFG_eval::getConstraints(const ADvector& x) {
    // 1. get the pose constraints
    std::vector<ADvector> constraints_list;

    ADvector pose_constraints;
    this->setPoseConstraints(x, &pose_constraints);
    constraints_list.push_back(pose_constraints);

    // 2. get the dynamic constraints
    ADvector dynamic_constraints;
    this->setDynamicConstraints(x, &dynamic_constraints);
    constraints_list.push_back(dynamic_constraints);

    // 3. get the control feasible constraints
    // ADvector control_constraints;
    // this->setControlFeasibleConstraints(x, &control_constraints);
    // constraints_list.push_back(control_constraints);

    // 4. get the avoidance constraints
    ADvector avoidanc_constraints;
    this->setAvoidanceConstraints(x, &avoidanc_constraints);
    constraints_list.push_back(avoidanc_constraints);

    // 5. cat all constraints
    ADvector constraints;
    constraints.clear();
    for (const auto& constraint_vec : constraints_list) {
        for (const auto& constraint_val : constraint_vec) { constraints.push_back(constraint_val); }
    }

    return constraints;
}

bool OBCAFG_eval::setPoseConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints) {
    if (constraints == nullptr) {
        LOG(WARNING) << "The pose constraints pointer is null.";
        return false;
    }

    std::vector<double> start_pose = {start_pose_ptr_->x, start_pose_ptr_->y, start_pose_ptr_->theta};
    // 1. initial pose constraint
    constraints->resize(pose_constraints_num_);
    (*constraints)[0] = start_pose_ptr_->x - x[VariableIndex::X];
    (*constraints)[1] = start_pose_ptr_->y - x[VariableIndex::Y];
    (*constraints)[2] = goal_pose_ptr_->x - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::X];
    (*constraints)[3] = goal_pose_ptr_->y - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::Y];
    (*constraints)[4] = start_pose_ptr_->theta - x[VariableIndex::THETA];
    (*constraints)[5] = goal_pose_ptr_->theta - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::THETA];
    (*constraints)[6] = x[VariableIndex::V];
    (*constraints)[7] = x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::V];
    return true;
}

bool OBCAFG_eval::setPoseConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub) {
    if (lb == nullptr || ub == nullptr) {
        LOG(WARNING) << "The lb or ub pointer is null.";
        return false;
    }
    // Keep a small tolerance for better numerical robustness in NLP solve.
    lb->resize(pose_constraints_num_);
    ub->resize(pose_constraints_num_);
    for (std::size_t i = 0; i < pose_constraints_num_; ++i) {
        (*lb)[i] = -0.0;
        (*ub)[i] = 0.0;
    }

    return true;
}

bool OBCAFG_eval::setDynamicConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints) {
    if (constraints == nullptr) {
        LOG(WARNING) << "The dynamic constraints pointer is null.";
        return false;
    }

    // 1. set the size of the constraints
    kinematic_model::VehicleParam vehicle_param(dynamic_model_ptr_->GetVehicleParam());
    constraints->resize((N_ - 1) * state_num_);
    // 2. set the lower and upper bounds
    for (std::size_t i = 0; i < (N_ - 1); ++i) {
        // 2. get the dynamic constraints
        CppAD::AD<double> f_x = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::X] -
                                x[i * (state_num_ + control_num_) + VariableIndex::X] -
                                x[i * (state_num_ + control_num_) + VariableIndex::V] *
                                    CppAD::cos(x[i * (state_num_ + control_num_) + VariableIndex::THETA]) * kDt;
        CppAD::AD<double> f_y = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::Y] -
                                x[i * (state_num_ + control_num_) + VariableIndex::Y] -
                                x[i * (state_num_ + control_num_) + VariableIndex::V] *
                                    CppAD::sin(x[i * (state_num_ + control_num_) + VariableIndex::THETA]) * kDt;
        CppAD::AD<double> f_theta = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::THETA] -
                                    x[i * (state_num_ + control_num_) + VariableIndex::THETA] -
                                    x[i * (state_num_ + control_num_) + VariableIndex::V] / vehicle_param.wheel_base() *
                                        CppAD::tan(x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE]) *
                                        kDt;
        CppAD::AD<double> f_v = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::V] -
                                x[i * (state_num_ + control_num_) + VariableIndex::V] -
                                x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION] * kDt;
        (*constraints)[i * state_num_ + VariableIndex::X]     = f_x;
        (*constraints)[i * state_num_ + VariableIndex::Y]     = f_y;
        (*constraints)[i * state_num_ + VariableIndex::THETA] = f_theta;
        (*constraints)[i * state_num_ + VariableIndex::V]     = f_v;
    }

    return true;
}

bool OBCAFG_eval::setDynamicConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub) {
    auto build_eq_bound = [&lb, &ub](std::size_t idx, double epsilon = 0.0) {
        (*lb)[idx] = -epsilon;
        (*ub)[idx] = epsilon;
    };
    lb->resize((N_ - 1) * state_num_);
    ub->resize((N_ - 1) * state_num_);
    for (std::size_t i = 0; i < (N_ - 1); ++i) {
        for (std::size_t j = 0; j < state_num_; ++j) { build_eq_bound(i * state_num_ + j); };
    }
    return true;
}

bool OBCAFG_eval::setControlFeasibleConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints) {
    if (constraints == nullptr) {
        LOG(WARNING) << "The constraints, lb or ub pointer is null.";
        return false;
    }
    constraints->resize((N_ - 1) * control_num_);

    // 1. set the control feasible constraints
    kinematic_model::VehicleParam vehicle_param(dynamic_model_ptr_->GetVehicleParam());
    for (std::size_t i = 0; i < (N_ - 1); ++i) {
        // 1.1 steering angle constraints
        CppAD::AD<double> steer_angle        = x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
        (*constraints)[i * control_num_ + 0] = steer_angle;
        CppAD::AD<double> acc                = x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
        (*constraints)[i * control_num_ + 1] = acc;
    }

    return true;
}

bool OBCAFG_eval::setControlFeasibleConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub) {
    std::size_t total_size = N_ * (state_num_ + control_num_) + lambda_num_ + mu_num_;
    lb->resize(total_size);
    ub->resize(total_size);
    auto build_ineq_bound = [&lb, &ub](std::size_t idx, double lower, double upper) {
        (*lb)[idx] = lower;
        (*ub)[idx] = upper;
    };
    // 1. set the bound for primal variables
    kinematic_model::VehicleParam vehicle_param(dynamic_model_ptr_->GetVehicleParam());
    for (std::size_t i = 0; i < N_; ++i) {
        build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::X, -kMaxValue, kMaxValue);
        build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::Y, -kMaxValue, kMaxValue);
        build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::V,
                         -vehicle_param.max_velocity(),
                         vehicle_param.max_velocity());
        build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::THETA, -kMaxValue, kMaxValue);
        if (i < N_ - 1) {
            build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE,
                             -vehicle_param.max_steer_angle(),
                             vehicle_param.max_steer_angle());
            build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::ACCELERATION,
                             -vehicle_param.max_acc(),
                             vehicle_param.max_acc());
        } else {
            // last control must be zero
            build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE, -kEpsilon, kEpsilon);
            build_ineq_bound(i * (state_num_ + control_num_) + VariableIndex::ACCELERATION, -kEpsilon, kEpsilon);
        }
    }

    // 2. set the bound for dual variables
    for (std::size_t i = N_ * (state_num_ + control_num_); i < lb->size(); ++i) {
        (*lb)[i] = 0.0;
        (*ub)[i] = kMaxValue;
    }

    return true;
}

bool OBCAFG_eval::setAvoidanceConstraints(const FG_eval::ADvector& x, FG_eval::ADvector* constraints) {
    if (constraints == nullptr) {
        LOG(WARNING) << "The avoidance obstacle constraints, lb or ub pointer is null.";
        return false;
    }

    // const std::size_t obstacle_bound_num = obstacle_A_.rows();
    // constraints->setZero(obstacle_bound_num * N_ * kVehicleBoundaryNum);
    // lb->setZero(obstacle_bound_num * N_ * kVehicleBoundaryNum);
    // ub->setZero(obstacle_bound_num * N_ * kVehicleBoundaryNum);
    double off_set =
        dynamic_model_ptr_->GetVehicleParam().length() / 2 - dynamic_model_ptr_->GetVehicleParam().rear_overhang();
    std::size_t dual_vari_start_idx = N_ * (state_num_ + control_num_);
    std::size_t lambda_start_idx    = dual_vari_start_idx;
    std::size_t mu_start_idx        = dual_vari_start_idx + N_ * obstacle_A_.rows();
    std::size_t constraint_idx      = 0;
    std::size_t obstacle_num        = obstable_bound_num_vec_.size();

    constraints->resize(4 * N_ * obstacle_num);

    // build variable vetor
    CppVecXd variable_x(x.size());
    auto     cpp_obstacle_A = obstacle_A_.cast<CppAD::AD<double>>();
    auto     cpp_obstacle_b = obstacle_b_.cast<CppAD::AD<double>>();
    for (std::size_t i = 0; i < x.size(); ++i) { variable_x[i] = x[i]; }
    for (std::size_t i = 0; i < N_; ++i) {
        // Note: For each time step i, indices are:
        // - primal variables: [i * (state_num_ + control_num_), (i+1) * (state_num_ + control_num_))
        // - lambda: [start_idx + i * total_obstacle_pts, ...)
        // - mu: [start_idx + N_ * total_obstacle_pts + i * kVehicleBoundaryNum * num_obstacles, ...)
        // For time step i, accumulate indices across previous time steps
        std::size_t lambda_idx       = lambda_start_idx + i * obstacle_A_.rows();
        std::size_t mu_idx           = mu_start_idx + i * obstacle_num * kVehicleBoundaryNum;
        std::size_t obstacle_a_start = 0;
        CppMatrixXd rotation_matrix;
        rotation_matrix.resize(2, 2);
        CppVecXd move_matrix;
        move_matrix.resize(2);
        OBCAFG_eval::getRotationMatrix(x[i * (state_num_ + control_num_) + VariableIndex::THETA], rotation_matrix);
        OBCAFG_eval::getMovementMatrix(x[i * (state_num_ + control_num_) + VariableIndex::X],
                                       x[i * (state_num_ + control_num_) + VariableIndex::Y],
                                       x[i * (state_num_ + control_num_) + VariableIndex::THETA],
                                       off_set,
                                       move_matrix);
        for (std::size_t j = 0; j < obstacle_num; ++j) {
            std::size_t obstacle_pts_num = obstable_bound_num_vec_[j];
            CppVecXd    lambda_j         = variable_x.segment(lambda_idx + obstacle_a_start, obstacle_pts_num);
            CppVecXd    mu_j             = variable_x.segment(mu_idx + j * kVehicleBoundaryNum, kVehicleBoundaryNum);
            CppMatrixXd A            = cpp_obstacle_A.block(obstacle_a_start, 0, obstacle_pts_num, obstacle_A_.cols());
            CppVecXd    b            = cpp_obstacle_b.segment(obstacle_a_start, obstacle_pts_num);
            CppVecXd    A_t_lambda   = A.transpose() * lambda_j;
            CppVecXd    stationarity = G_.transpose() * mu_j + rotation_matrix.transpose() * A_t_lambda;
            (*constraints)[constraint_idx]     = (A_t_lambda.transpose() * A_t_lambda)(0);
            (*constraints)[constraint_idx + 1] = stationarity(0);
            (*constraints)[constraint_idx + 2] = stationarity(1);
            (*constraints)[constraint_idx + 3] =
                (lambda_j.transpose() * (A * move_matrix - b) - mu_j.transpose() * g_)(0);
            constraint_idx += 4;
            obstacle_a_start += obstacle_pts_num;
        }
    }

    return true;
}

bool OBCAFG_eval::setAvoidanceConstraintsBound(IpoptSolver::Dvector* lb, IpoptSolver::Dvector* ub) {
    std::size_t obstacle_num = obstable_bound_num_vec_.size();
    // Safe distance to maintain from obstacles (meters)
    double safe_dist = kSafeDist;   // Use the predefined safe distance (0.05m)
    lb->resize(4 * N_ * obstacle_num);
    ub->resize(4 * N_ * obstacle_num);
    std::size_t constraint_idx = 0;
    for (std::size_t i = 0; i < N_; ++i) {
        for (std::size_t j = 0; j < obstable_bound_num_vec_.size(); ++j) {
            // Constraint 0: ||A^T * lambda||^2 in [0, 1]
            (*lb)[constraint_idx] = 0.0;
            (*ub)[constraint_idx] = 1.0;
            // Constraint 1-2: stationarity equalities
            (*lb)[constraint_idx + 1] = 0.0;
            (*ub)[constraint_idx + 1] = 0.0;
            (*lb)[constraint_idx + 2] = 0.0;
            (*ub)[constraint_idx + 2] = 0.0;
            // Constraint 3: separation condition
            (*lb)[constraint_idx + 3] = safe_dist;
            (*ub)[constraint_idx + 3] = kMaxValue;
            constraint_idx += 4;
        }
    }
    return true;
}

bool OBCAFG_eval::getObstacleBound(const std::shared_ptr<map::Map>& map_ptr) {
    // 1. clear the obstacle_A and obstacle_b
    obstacle_A_.resize(0, 2);
    obstacle_b_.resize(0);
    // 2. get obstacle from map to build hyperlane
    if (map_ptr == nullptr) {
        LOG(WARNING) << "The map pointer is null.";
        return false;
    }

    const auto& obs_list = map_ptr->GetObsList();
    obstable_bound_num_vec_.clear();
    for (std::size_t i = 0; i < obs_list.size(); ++i) {
        Eigen::MatrixXd obstacle_A_tmp;
        Eigen::VectorXd obstacle_b_tmp;
        if (!OBCAFG_eval::getHyperLane(obs_list[i], obstacle_A_tmp, obstacle_b_tmp, i)) {
            LOG(WARNING) << "Failed to get the hyperlane for obstacle_" << i;
            return false;
        }
        obstable_bound_num_vec_.push_back(obstacle_b_tmp.size());
        // cat the obstacle_A and obstacle_b
        obstacle_A_.conservativeResize(obstacle_A_.rows() + obstacle_A_tmp.rows(), 2);
        obstacle_b_.conservativeResize(obstacle_b_.size() + obstacle_b_tmp.size());
        obstacle_A_.block(obstacle_A_.rows() - obstacle_A_tmp.rows(), 0, obstacle_A_tmp.rows(), obstacle_A_tmp.cols()) =
            obstacle_A_tmp;
        obstacle_b_.segment(obstacle_b_.size() - obstacle_b_tmp.size(), obstacle_b_tmp.size()) = obstacle_b_tmp;
    }

    return true;
}

}   // namespace backend
}   // namespace planning