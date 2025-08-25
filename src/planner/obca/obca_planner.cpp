#include "obca_planner.h"

namespace planning {
namespace backend {

bool OBCASolver::Solve(const vehicle_model::sdv_traj& init_trajectory, const vehicle_model::VehiclePose& start_pose,
                       const vehicle_model::VehiclePose& goal_pose) {
    // 1. get the initial variables
    Eigen::MatrixXd init_states;
    Eigen::MatrixXd obstacle_A;
    Eigen::VectorXd obstacle_b;

    // 2. get obstacle from map to build hyperlane
    if (map_ptr_ == nullptr) {
        LOG(WARNING) << "The map pointer is null.";
        return false;
    }
    const auto& obs_list = map_ptr_->GetObsList();

    for (std::size_t i = 0; i < obs_list.size(); ++i) {
        Eigen::MatrixXd obstacle_A_tmp;
        Eigen::VectorXd obstacle_b_tmp;
        if (!getHyperLane(obs_list[i], &obstacle_A_tmp, &obstacle_b_tmp, i)) {
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
};

bool OBCASolver::setInitVariable() {
    // Set the initial variables for the optimization problem
    states_result_.clear();
    controls_result_.clear();
    return true;
};

bool OBCASolver::initializeParameters(const kinematic_model::VehicleParam& vehicle_param) {
    offset_ = vehicle_param.length() / 2 - vehicle_param.rear_overhang();
    // Initialize the parameters for the OBCA algorithm
    // Set the vehicle parameters
    G_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
    g_ << vehicle_param.length() / 2, vehicle_param.width() / 2, vehicle_param.length() / 2, vehicle_param.width() / 2;

    return true;
};

bool OBCASolver::getHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd* obstacle_A,
                              Eigen::VectorXd* obstacle_b, std::size_t i) {
    // 1. compute the hyperlane for obstacle
    obstacle_A->resize(obstacle.num_points(), 2);
    obstacle_b->resize(obstacle.num_points());

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
        (*obstacle_A)(j, 0) = normal_vector.x();
        (*obstacle_A)(j, 1) = normal_vector.y();
        (*obstacle_b)(j)    = normal_vector.InnerProd(line_segments[j].start());
    }
    LOG(INFO) << "obstacle_" << i << " hyperlane is generated successfully.";

    return true;
}

bool OBCASolver::getRotationMatrix(const double theta, Eigen::Matrix2d* R) {
    // get the rotation matrix
    if (R == nullptr) {
        LOG(WARNING) << "The rotation matrix is null.";
        return false;
    }
    (*R)(0, 0) = std::cos(theta);
    (*R)(0, 1) = -std::sin(theta);
    (*R)(1, 0) = std::sin(theta);
    (*R)(1, 1) = std::cos(theta);

    return true;
}

bool OBCASolver::getMovementMatrix(const double x, const double y, const double theta, Eigen::Matrix<double, 2, 1>* t) {
    // get the movement matrix
    if (t == nullptr) {
        LOG(WARNING) << "The movement matrix is null.";
        return false;
    }
    (*t)(0, 0) = x + offset_ * std::cos(theta);
    (*t)(1, 0) = y + offset_ * std::sin(theta);

    return true;
}



}   // namespace backend
}   // namespace planning