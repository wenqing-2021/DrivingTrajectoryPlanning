#include "piece_wise_jerk.h"
#include "OsqpEigen/OsqpEigen.h"
#include "math/math_utils.h"

namespace planning {
namespace backend {

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(const params::PiecewiseJerkParams& pwj_params, double dt)
    : pwj_params_(pwj_params) {
    // 1. set the piecewise jerk speed optimizer config
    // setParams(config);
    // 2. set the piecewise jerk speed optimizer solver
    qp_solver_ptr_ = std::make_unique<QPSolver>();
    // 3. set the time step
    kDt  = dt;
    kDt2 = kDt * kDt;
};

bool PiecewiseJerkSpeedOptimizer::Optimize(const std::vector<Eigen::Vector3d>& path,
                                           const vehicle_model::VehiclePose&   vehicle_pose) {
    result_traj_.clear();
    std::vector<std::vector<Eigen::Vector3d>> shift_seg_paths;   // [x, y, v]
    // 1. get the shift segment paths
    if (!getShiftSegPaths(path, &shift_seg_paths)) {
        LOG(WARNING) << "Failed to get the shift segment paths";
        return false;
    }
    LOG(INFO) << "The shift segment paths size is: " << shift_seg_paths.size();
    segment_num_ = shift_seg_paths.size();
    // 2. optimize the speed for each segment and fill the result path
    double init_path_head =
        common::math::NormalizeAngle(std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x()));
    double speed_direct = getSpeedDirect(init_path_head, vehicle_pose.theta);
    for (std::size_t i = 0; i < shift_seg_paths.size(); ++i) {
        // 2.1 optimize the speed
        std::vector<Eigen::Vector3d> shift_seg_traj = shift_seg_paths[i];
        if (!optimizeSpeed(shift_seg_traj)) {
            // 2.2 if optimize failed, return
            LOG(WARNING) << "Failed to optimize the speed for segment " << i;
            return false;
        } else {
            LOG(INFO) << "The speed optimization for segment " << i << " is success";
            const Eigen::VectorXd* const res = qp_solver_ptr_->GetResult();
            // 2.3 if optimize success, fill the result path
            std::size_t start_j = 0;
            if (i > 0) {
                start_j = 1;   // skip the first point of the segment if it is not the first segment
            }
            for (std::size_t j = start_j; j < shift_seg_traj.size(); ++j) {
                shift_seg_traj[j].z() = speed_direct * (*res)(j + shift_seg_traj.size());
                // insert x, y, theta v into the result traj
                Eigen::Vector4d traj_point;
                traj_point.x() = shift_seg_paths[i][j].x();
                traj_point.y() = shift_seg_paths[i][j].y();
                traj_point.z() = shift_seg_paths[i][j].z();   // set theta
                traj_point.w() = shift_seg_traj[j].z();       // set v
                result_traj_.push_back(traj_point);
            }
            speed_direct = -speed_direct;
        }
    }

    return true;
};

bool PiecewiseJerkSpeedOptimizer::getShiftSegPaths(const std::vector<Eigen::Vector3d>&        path,
                                                   std::vector<std::vector<Eigen::Vector3d>>* shift_seg_paths) {
    // 1. convert the vector to the eigen matrix with path vector
    if (path.size() < 2) {
        LOG(WARNING) << "The path size is less than 2";
        return false;
    };
    Eigen::MatrixXd path_vector(path.size() - 1, 2);
    for (std::size_t i = 1; i < path.size(); ++i) {
        path_vector(i - 1, 0) = path[i].x() - path[i - 1].x();
        path_vector(i - 1, 1) = path[i].y() - path[i - 1].y();
    }
    // 2. Compute the dot product of consecutive vectors
    Eigen::VectorXd dot_products =
        (path_vector.topRows(path_vector.rows() - 1).array() * path_vector.bottomRows(path_vector.rows() - 1).array())
            .rowwise()
            .sum();
    // 3. Find each segment paths
    shift_seg_paths->clear();
    int start_index = 0;
    for (int i = 0; i < dot_products.size(); ++i) {
        if (dot_products(i) < 0) {
            std::vector<Eigen::Vector3d> seg_path;
            for (int j = start_index; j <= i + 1; ++j) { seg_path.push_back(path[j]); }
            shift_seg_paths->push_back(seg_path);
            start_index = i + 1;
        }
    };
    // 4. Add the last segment path
    std::vector<Eigen::Vector3d> Last_seg_path;
    for (int j = start_index; j < path.size(); ++j) { Last_seg_path.push_back(path[j]); }
    shift_seg_paths->push_back(Last_seg_path);

    return true;
}

double PiecewiseJerkSpeedOptimizer::getSpeedDirect(const double init_path_head, const double vehicle_head) {
    if (common::math::AngleDiff(vehicle_head, init_path_head) > M_PI_2) {
        return -1.0;
    } else {
        return 1.0;
    }
}

bool PiecewiseJerkSpeedOptimizer::optimizeSpeed(const std::vector<Eigen::Vector3d>& path) {
    // REF: https://zhuanlan.zhihu.com/p/666460606
    // 1. set varialbes_num and initialize the variables
    std::size_t path_point_num = path.size();
    if (!initVariables(path)) {
        LOG(WARNING) << "Failed to initialize the variables";
        return false;
    };
    // 2. set the number of variables [s, v, a]
    int var_num = path_point_num * 3;
    qp_solver_ptr_->setVariableNums(var_num);
    int cons_num = 3 * path_point_num + 2 * (path_point_num - 1) + 3;
    qp_solver_ptr_->setConstraintNums(cons_num);

    // 3. set the hessian and gradient matrix
    hessian_matrix_.resize(var_num, var_num);
    hessian_matrix_.setZero();
    gradient_.resize(var_num);
    gradient_.setZero();
    if (!initHessianGradient(path)) {
        LOG(WARNING) << "Failed to initialize the hessian and gradient matrix";
        return false;
    }

    // 4. set the linear matrix and lower && upper bound
    linear_matrix_.resize(cons_num, var_num);
    linear_matrix_.setZero();
    lower_bound_.resize(cons_num);
    lower_bound_.setZero();
    upper_bound_.resize(cons_num);
    upper_bound_.setZero();
    if (!initBounds(path)) {
        LOG(WARNING) << "Failed to initialize the bounds";
        return false;
    }
    Eigen::SparseMatrix<double> hessian_matrix = hessian_matrix_.sparseView();
    Eigen::SparseMatrix<double> linear_matrix  = linear_matrix_.sparseView();
    // 5. solve the problem
    if (!qp_solver_ptr_->Solve(hessian_matrix, gradient_, linear_matrix, lower_bound_, upper_bound_)) {
        LOG(WARNING) << "Failed to solve the problem";
        return false;
    }

    return true;
}

bool PiecewiseJerkSpeedOptimizer::initVariables(const std::vector<Eigen::Vector3d>& path) {
    // use lambda function to caclulate the cumulative distance
    auto calcCumulativeDistance = [](const std::vector<Eigen::Vector3d>& path) {
        std::vector<double> cumulative_distance(path.size(), 0.0);
        for (std::size_t i = 1; i < path.size(); ++i) {
            cumulative_distance[i] = cumulative_distance[i - 1] + std::sqrt(std::pow(path[i].x() - path[i - 1].x(), 2) +
                                                                            std::pow(path[i].y() - path[i - 1].y(), 2));
        }
        return cumulative_distance;
    };
    // 1. check the path length
    if (path.size() < 2) {
        LOG(WARNING) << "The path size is less than 2";
        return false;
    };
    // 2. calculate the cumulative distance
    std::vector<double> cumulative_distance = calcCumulativeDistance(path);
    if (cumulative_distance.back() < 1e-6) {
        LOG(WARNING) << "The path length is less than 1e-6";
        return false;
    };
    double init_v = pwj_params_.max_v();
    double init_a = 0.0;
    // 3. initialize the variables
    std::size_t path_point_num = path.size();
    init_variables_.resize(path_point_num * 3);
    init_variables_.setZero();
    for (std::size_t i = 0; i < path_point_num; ++i) {
        init_variables_(i) = cumulative_distance[i];
        if (i == 0 || i == path_point_num - 1) {
            init_variables_(path_point_num + i)     = 0.0;
            init_variables_(2 * path_point_num + i) = 0.0;
        } else {
            init_variables_(path_point_num + i)     = init_v;
            init_variables_(2 * path_point_num + i) = init_a;
        }
    }

    return true;
}

bool PiecewiseJerkSpeedOptimizer::initHessianGradient(const std::vector<Eigen::Vector3d>& path) {
    std::size_t path_point_num = path.size();
    // 1. set the hessian matrix
    // 1.1 set the hessian matrix for s
    const double    w_s_ref   = pwj_params_.w_s_ref();
    Eigen::MatrixXd s_hessian = w_s_ref * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    s_hessian(path_point_num - 1, path_point_num - 1) += pwj_params_.w_s_end();

    // 1.2 set the hessian matrix for v
    const double    w_v_ref   = pwj_params_.w_v_ref();
    Eigen::MatrixXd v_hessian = w_v_ref * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    v_hessian(path_point_num - 1, path_point_num - 1) += pwj_params_.w_v_end();

    // 1.3 set the hessian matrix for a
    const double    w_a       = pwj_params_.w_acc();
    const double    w_jerk    = pwj_params_.w_jerk();
    Eigen::MatrixXd a_hessian = (w_a + w_jerk / kDt2) * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    a_hessian.diagonal().segment(1, path_point_num - 1).array() += w_jerk / kDt2;
    a_hessian(path_point_num - 1, path_point_num - 1) += pwj_params_.w_a_end();
    a_hessian.diagonal(1).array() += -w_jerk / kDt2;
    a_hessian.diagonal(-1).array() += -w_jerk / kDt2;

    hessian_matrix_.block(0, 0, path_point_num, path_point_num)                                   = s_hessian;
    hessian_matrix_.block(path_point_num, path_point_num, path_point_num, path_point_num)         = v_hessian;
    hessian_matrix_.block(2 * path_point_num, 2 * path_point_num, path_point_num, path_point_num) = a_hessian;

    // 2. set the gradient matrix
    for (std::size_t i = 0; i < path_point_num; ++i) {
        gradient_(i)                  = -pwj_params_.w_s_ref() * init_variables_(i);
        gradient_(path_point_num + i) = -pwj_params_.w_v_ref() * init_variables_(path_point_num + i);
        if (i == path_point_num - 1) {
            gradient_(i) += -pwj_params_.w_s_end() * init_variables_(i);
            gradient_(path_point_num + i) += -pwj_params_.w_v_end() * init_variables_(path_point_num + i);
            gradient_(2 * path_point_num + i) += -pwj_params_.w_a_end() * init_variables_(2 * path_point_num + i);
        }
    }

    return true;
}

bool PiecewiseJerkSpeedOptimizer::initBounds(const std::vector<Eigen::Vector3d>& path) {
    // u_lower <= Ax <= u_upper
    // 1. set the linear matrix
    std::size_t path_point_num = path.size();
    if (path_point_num < 2) {
        LOG(WARNING) << "The path size is less than 2";
        return false;
    };
    Eigen::MatrixXd linear_matrix_bound       = Eigen::MatrixXd::Identity(3 * path_point_num, 3 * path_point_num);
    Eigen::MatrixXd linear_matrix_system      = Eigen::MatrixXd::Zero(2 * (path_point_num - 1), 3 * path_point_num);
    Eigen::MatrixXd linear_matrix_init        = Eigen::MatrixXd::Zero(3, 3 * path_point_num);
    linear_matrix_init(0, 0)                  = 1;
    linear_matrix_init(1, path_point_num)     = 1;
    linear_matrix_init(2, 2 * path_point_num) = 1;

    // 2. set the system linear matrix
    // 2.1 set s
    Eigen::MatrixXd s_1     = Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    s_1.diagonal(1).array() = -1;
    s_1.conservativeResize(path_point_num - 1, path_point_num);
    Eigen::MatrixXd s_2 = Eigen::MatrixXd::Zero(path_point_num - 1, path_point_num);
    linear_matrix_system.block(0, 0, path_point_num - 1, path_point_num)                  = s_1;
    linear_matrix_system.block(path_point_num - 1, 0, path_point_num - 1, path_point_num) = s_2;
    // 2.2 set v
    Eigen::MatrixXd v_1 = kDt / 2 * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    v_1.conservativeResize(path_point_num - 1, path_point_num);
    linear_matrix_system.block(0, path_point_num, path_point_num - 1, path_point_num)                  = v_1;
    linear_matrix_system.block(path_point_num - 1, path_point_num, path_point_num - 1, path_point_num) = s_1;
    // 2.3 set a
    Eigen::MatrixXd a_1     = kDt2 / 3 * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    a_1.diagonal(1).array() = kDt2 / 6;
    a_1.conservativeResize(path_point_num - 1, path_point_num);
    Eigen::MatrixXd a_2     = kDt / 2 * Eigen::MatrixXd::Identity(path_point_num, path_point_num);
    a_2.diagonal(1).array() = kDt / 2;
    a_2.conservativeResize(path_point_num - 1, path_point_num);
    linear_matrix_system.block(0, 2 * path_point_num, path_point_num - 1, path_point_num)                  = a_1;
    linear_matrix_system.block(path_point_num - 1, 2 * path_point_num, path_point_num - 1, path_point_num) = a_2;
    linear_matrix_.block(0, 0, 3 * path_point_num, 3 * path_point_num)                        = linear_matrix_bound;
    linear_matrix_.block(3 * path_point_num, 0, 2 * (path_point_num - 1), 3 * path_point_num) = linear_matrix_system;
    linear_matrix_.block(3 * path_point_num + 2 * (path_point_num - 1), 0, 3, 3 * path_point_num) = linear_matrix_init;

    // 3. set the lower and upper bound
    double max_s = init_variables_(path_point_num - 1);
    double min_s = 0.0;
    double max_v = pwj_params_.max_v();
    double min_v = pwj_params_.min_v();
    double max_a = pwj_params_.max_acc();
    double min_a = pwj_params_.min_acc();

    lower_bound_.segment(0, path_point_num).array()                            = 0.0;         // lower s
    lower_bound_.segment(path_point_num, path_point_num).array()               = min_v;       // lower v
    lower_bound_.segment(2 * path_point_num, path_point_num).array()           = min_a;       // lower a
    lower_bound_.segment(3 * path_point_num, 2 * (path_point_num - 1)).array() = 0.0;         // equal constraint system
    lower_bound_.segment(2 * path_point_num - 1, 1).array()                    = -kEpsilon;   // last v
    lower_bound_.segment(3 * path_point_num - 1, 1).array()                    = -kEpsilon;   // last a
    Eigen::Vector3d init_value;
    init_value(0) = init_variables_(0);
    init_value(1) = init_variables_(path_point_num);
    init_value(2) = init_variables_(2 * path_point_num);

    lower_bound_.segment(3 * path_point_num + 2 * (path_point_num - 1), 2).array() =
        init_value.segment(0, 2).array() - kEpsilon;                                              // initial s, v
    lower_bound_.segment(3 * path_point_num + 2 * (path_point_num - 1) + 2, 1).array() = min_a;   // initial a

    upper_bound_.segment(0, path_point_num).array()                            = max_s;   // upper s
    upper_bound_.segment(path_point_num, path_point_num).array()               = max_v;   // upper v
    upper_bound_.segment(2 * path_point_num, path_point_num).array()           = max_a;   // upper a
    upper_bound_.segment(3 * path_point_num, 2 * (path_point_num - 1)).array() = 0.0;     // equal constraint system
    upper_bound_.segment(3 * path_point_num + 2 * (path_point_num - 1), 2).array() =
        init_value.segment(0, 2).array() + kEpsilon;                                                 // initial s, v
    upper_bound_.segment(3 * path_point_num + 2 * (path_point_num - 1) + 2, 1).array() = max_a;      // initial a
    upper_bound_.segment(2 * path_point_num - 1, 1).array()                            = kEpsilon;   // last v
    upper_bound_.segment(3 * path_point_num - 1, 1).array()                            = kEpsilon;   // last a

    return true;
}
}   // namespace backend
}   // namespace planning
