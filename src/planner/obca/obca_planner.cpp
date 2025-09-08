#include "obca_planner.h"

namespace planning {
namespace backend {

bool OBCASolver::Process(const vehicle_model::sdv_path& init_path, const vehicle_model::VehiclePose& start_pose,
                         const vehicle_model::VehiclePose& goal_pose) {
    OBCAFG_eval fg_eval;
    // 1. get the initial variables
    if (setInitVariable(init_path, &fg_eval)) {
        LOG(INFO) << "The initial variables are set successfully.";
    } else {
        LOG(WARNING) << "Failed to set the initial variables.";
        return false;
    }

    // 2. set the cost function

    // 3. set the constraints

    return true;
};

bool OBCASolver::setInitVariable(const vehicle_model::sdv_path& init_path, OBCAFG_eval* fg_eval) {
    // 1. Set the initial variables for the optimization problem.
    // NOTE: initial path including the start pose and the goal pose
    states_result_.clear();
    controls_result_.clear();
    N_ = init_path.size();
    x0_.resize(N_ * (state_num_ + control_num_));
    for (std::size_t i = 0; i < N_; ++i) {
        x0_[i * (state_num_ + control_num_) + VariableIndex::X]     = init_path[i].x;
        x0_[i * (state_num_ + control_num_) + VariableIndex::Y]     = init_path[i].y;
        x0_[i * (state_num_ + control_num_) + VariableIndex::THETA] = init_path[i].theta;
        x0_[i * (state_num_ + control_num_) + VariableIndex::V]     = 0.0;   //
        x0_[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE] =
            0.0;   // hack the initial steering angle to be zero
        x0_[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION] =
            0.0;   // hack the initial acceleration to be zero
    }
    // 2. set the dual variables
    // 2.1 get the obstacle from map
    if (map_ptr_ == nullptr) {
        LOG(WARNING) << "The map pointer is null.";
        return false;
    }
    const auto& obs_list         = map_ptr_->GetObsList();
    std::size_t obstacle_num     = obs_list.size();
    std::size_t obstacle_num_pts = 0;
    for (const auto& obs : obs_list) { obstacle_num_pts += obs.num_points(); }
    // 2.2 set the dual variables
    Eigen::VectorXd mu0_(obstacle_num_pts * N_);
    Eigen::VectorXd lambda0_(obstacle_num * N_ * kVehicleBoundaryNum);
    mu0_.setOnes();
    mu0_ *= 0.1;
    lambda0_.setOnes();
    lambda0_ *= 0.1;
    // 3. set the initial value
    x0_.conservativeResize(x0_.size() + mu0_.size() + lambda0_.size());
    x0_.tail(mu0_.size())     = mu0_;
    x0_.tail(lambda0_.size()) = lambda0_;

    fg_eval->setInitParameters(x0_, N_, state_num_, control_num_);

    return true;
};

bool OBCASolver::initializeParameters(const kinematic_model::VehicleParam& vehicle_param) {
    offset_      = vehicle_param.length() / 2 - vehicle_param.rear_overhang();
    state_num_   = vehicle_model::KinematicModel::GetStateSize();     // 4: [x, y, theta, v]
    control_num_ = vehicle_model::KinematicModel::GetControlSize();   // 2: [sigma, a]
    // Initialize the parameters for the OBCA algorithm
    // Set the vehicle parameters
    G_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
    g_ << vehicle_param.length() / 2, vehicle_param.width() / 2, vehicle_param.length() / 2, vehicle_param.width() / 2;
    return true;
};

bool OBCASolver::getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
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

bool OBCASolver::getRotationMatrix(const double theta, Eigen::Matrix2d& R) {
    R.resize(2, 2);
    R(0, 0) = std::cos(theta);
    R(0, 1) = -std::sin(theta);
    R(1, 0) = std::sin(theta);
    R(1, 1) = std::cos(theta);

    return true;
}

bool OBCASolver::getMovementMatrix(const double x, const double y, const double theta, Eigen::Matrix<double, 2, 1>& t) {
    // get the movement matrix
    t.resize(2, 1);
    t(0, 0) = x + offset_ * std::cos(theta);
    t(1, 0) = y + offset_ * std::sin(theta);

    return true;
}

CppAD::AD<double> OBCAFG_eval::getCostFunction(const ADvector& x) {
    CppAD::AD<double> cost = 0.0;
    for (std::size_t i = 0; i < N_; ++i) {
        // 1. cost for ref_X
        CppAD::AD<double> x_pos = x[i * (state_num_ + control_num_) + VariableIndex::X];
        CppAD::AD<double> y_pos = x[i * (state_num_ + control_num_) + VariableIndex::Y];
        cost += (x_pos - ref_X_[i * 2]) * (x_pos - ref_X_[i * 2]);
        cost += (y_pos - ref_X_[i * 2 + 1]) * (y_pos - ref_X_[i * 2 + 1]);
        // 2. cost for control delta
        if (i < N_ - 1) {
            CppAD::AD<double> steer_angle1  = x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
            CppAD::AD<double> steer_angle2  = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
            CppAD::AD<double> acceleration1 = x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
            CppAD::AD<double> acceleration2 = x[(i + 1) * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
            cost += (steer_angle2 - steer_angle1) * (steer_angle2 - steer_angle1);
            cost += (acceleration2 - acceleration1) * (acceleration2 - acceleration1);
        }
    }

    return cost;
}

bool OBCAFG_eval::getPoseConstraints(const ADvector& start_pose, const ADvector& goal_pose, const ADvector& x,
                                     FG_eval::ADvector* constraints, FG_eval::ADvector* lb, FG_eval::ADvector* ub) {
    if (constraints == nullptr || lb == nullptr || ub == nullptr) {
        LOG(WARNING) << "The constraints, lb or ub pointer is null.";
        return false;
    }
    double kPositionTol = 0.05;   // m
    double kThetaTol    = 0.01;   // rad
    // 1. initial pose constraint
    constraints->resize(4);
    (*constraints)[0] = start_pose[0] - x[VariableIndex::X];
    (*constraints)[1] = start_pose[1] - x[VariableIndex::Y];
    (*constraints)[2] = goal_pose[0] - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::X];
    (*constraints)[3] = goal_pose[1] - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::Y];
    (*constraints)[4] = start_pose[2] - x[VariableIndex::THETA];
    (*constraints)[5] = goal_pose[2] - x[(N_ - 1) * (state_num_ + control_num_) + VariableIndex::THETA];

    // 2. set the lower and upper bounds
    lb->resize(6);
    ub->resize(6);
    for (std::size_t i = 0; i < 6; ++i) {
        if (i < 4) {
            (*lb)[i] = -kPositionTol;
            (*ub)[i] = kPositionTol;
        } else {
            (*lb)[i] = -kThetaTol;
            (*ub)[i] = kThetaTol;
        }
    }

    return true;
}

bool OBCAFG_eval::getDynamicConstraints(const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb,
                                        FG_eval::ADvector*                                    ub,
                                        const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr) {
    if (constraints == nullptr || lb == nullptr || ub == nullptr) {
        LOG(WARNING) << "The constraints, lb or ub pointer is null.";
        return false;
    }
    auto build_eq_bound = [&lb, &ub](std::size_t idx, double epsilon = 1e-3) {
        (*lb)[idx] = 0.0 - epsilon;
        (*ub)[idx] = 0.0 + epsilon;
    };
    // 1. set the size of the constraints
    kinematic_model::VehicleParam vehicle_param(dynamic_model_ptr->GetVehicleParam());
    constraints->resize((N_ - 1) * state_num_);
    lb->resize((N_ - 1) * state_num_);
    ub->resize((N_ - 1) * state_num_);
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
        for (std::size_t j = 0; j < state_num_; ++j) { build_eq_bound(i * state_num_ + j); }
    }

    return true;
}

bool OBCAFG_eval::getControlFeasibleConstraints(
    const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb, FG_eval::ADvector* ub,
    const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr) {
    if (constraints == nullptr || lb == nullptr || ub == nullptr) {
        LOG(WARNING) << "The constraints, lb or ub pointer is null.";
        return false;
    }
    constraints->clear();
    lb->clear();
    ub->clear();
    constraints->resize((N_ - 1) * control_num_);
    lb->resize((N_ - 1) * control_num_);
    ub->resize((N_ - 1) * control_num_);
    auto build_ineq_bound = [&lb, &ub](std::size_t idx, double lower, double upper) {
        (*lb)[idx] = lower;
        (*ub)[idx] = upper;
    };
    // 1. set the control feasible constraints
    kinematic_model::VehicleParam vehicle_param(dynamic_model_ptr->GetVehicleParam());
    for (std::size_t i = 0; i < (N_ - 1); ++i) {
        // 1.1 steering angle constraints
        CppAD::AD<double> steer_angle        = x[i * (state_num_ + control_num_) + VariableIndex::STEER_ANGLE];
        (*constraints)[i * control_num_ + 0] = steer_angle;
        build_ineq_bound(i * control_num_ + 0, -vehicle_param.max_steer_angle(), vehicle_param.max_steer_angle());
        CppAD::AD<double> acc                = x[i * (state_num_ + control_num_) + VariableIndex::ACCELERATION];
        (*constraints)[i * control_num_ + 1] = acc;
        build_ineq_bound(i * control_num_ + 1, -vehicle_param.max_acc(), vehicle_param.max_acc());
    }

    return true;
}

bool OBCAFG_eval::getAvoidanceConstraints(const ADvector& x, FG_eval::ADvector* constraints, FG_eval::ADvector* lb,
                                          FG_eval::ADvector* ub) {
    if (constraints == nullptr || lb == nullptr || ub == nullptr) {
        LOG(WARNING) << "The constraints, lb or ub pointer is null.";
        return false;
    }



    return true;
}

bool OBCASolver::getObstacleBound(const std::shared_ptr<map::Map> map_ptr, Eigen::MatrixXd& obstacle_A,
                                  Eigen::VectorXd& obstacle_b) {

    // 2. get obstacle from map to build hyperlane
    if (map_ptr == nullptr) {
        LOG(WARNING) << "The map pointer is null.";
        return false;
    }
    const auto& obs_list = map_ptr->GetObsList();

    for (std::size_t i = 0; i < obs_list.size(); ++i) {
        Eigen::MatrixXd obstacle_A_tmp;
        Eigen::VectorXd obstacle_b_tmp;
        if (!OBCASolver::getHyperLane(obs_list[i], obstacle_A_tmp, obstacle_b_tmp, i)) {
            LOG(WARNING) << "Failed to get the hyperlane for obstacle_" << i;
            return false;
        }
        // cat the obstacle_A and obstacle_b
        if (i == 0) {
            obstacle_A = obstacle_A_tmp;
            obstacle_b = obstacle_b_tmp;
        } else {
            Eigen::MatrixXd obstacle_A_new(obstacle_A.rows() + obstacle_A_tmp.rows(), 2);
            Eigen::VectorXd obstacle_b_new(obstacle_b.size() + obstacle_b_tmp.size());
            obstacle_A_new << obstacle_A, obstacle_A_tmp;
            obstacle_b_new << obstacle_b, obstacle_b_tmp;
            obstacle_A = obstacle_A_new;
            obstacle_b = obstacle_b_new;
        }
    }

    return true;
}

}   // namespace backend
}   // namespace planning