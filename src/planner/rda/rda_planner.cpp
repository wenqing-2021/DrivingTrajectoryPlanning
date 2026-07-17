/*
 * @Author: wenqing-2021 yuansj@hnu.edu.cn
 * @Date: 2026-07-05 07:27:48
 * @LastEditors: wenqing-2021 yuansj@hnu.edu.cn
 * @LastEditTime: 2026-07-17 05:38:52
 * @FilePath: /AutomatedPark/src/planner/rda/rda_planner.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 */
#include "rda_planner.h"
#include "logger.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace planning {
namespace backend {

RDASolver::RDASolver(const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
                     const std::shared_ptr<map::Map>& map_ptr, const params::RDAParams& rda_params, double dt)
    : dynamic_model_ptr_(dynamic_model_ptr)
    , map_ptr_(map_ptr)
    , rda_params_(rda_params)
    , dt_(dt) {

    L_ = dynamic_model_ptr_->GetVehicleParam().wheel_base();

    // Vehicle bounding box constraints: G * c <= h
    G_vehicle_.resize(4, 2);
    G_vehicle_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;

    double half_length = dynamic_model_ptr_->GetVehicleParam().length() / 2.0;
    double half_width  = dynamic_model_ptr_->GetVehicleParam().width() / 2.0;

    h_vehicle_.resize(4);
    h_vehicle_ << half_length, half_width, half_length, half_width;
}

void RDASolver::computeLinearizedDynamics(double v, double delta, double theta, Eigen::MatrixXd& A, Eigen::MatrixXd& B,
                                          Eigen::VectorXd& C) {
    // X = [x, y, theta, v]^T
    // U = [delta, a]^T
    // x_{k+1} = x_k + v * cos(theta + beta) * dt
    // y_{k+1} = y_k + v * sin(theta + beta) * dt
    // theta_{k+1} = theta_k + (v / L) * sin(beta) * dt
    // v_{k+1} = v_k + a * dt
    // beta = arctan(0.5 * tan(delta))

    double beta = std::atan(0.5 * std::tan(delta));
    double cb   = std::cos(beta);
    double sb   = std::sin(beta);
    double c_tb = std::cos(theta + beta);
    double s_tb = std::sin(theta + beta);

    A.setIdentity(4, 4);
    A(0, 2) = -v * s_tb * dt_;
    A(0, 3) = c_tb * dt_;
    A(1, 2) = v * c_tb * dt_;
    A(1, 3) = s_tb * dt_;
    A(2, 3) = (1.0 / L_) * sb * dt_;

    // derivative of beta with respect to delta
    // d(beta)/d(delta) = (0.5 * sec^2(delta)) / (1 + 0.25 * tan^2(delta))
    double sec2         = 1.0 / (std::cos(delta) * std::cos(delta));
    double dbeta_ddelta = (0.5 * sec2) / (1.0 + 0.25 * std::tan(delta) * std::tan(delta));

    B.setZero(4, 2);
    B(0, 0) = -v * s_tb * dbeta_ddelta * dt_;
    B(1, 0) = v * c_tb * dbeta_ddelta * dt_;
    B(2, 0) = (v / L_) * cb * dbeta_ddelta * dt_;
    B(3, 1) = dt_;

    Eigen::VectorXd X(4);
    X << 0.0, 0.0, theta, v;
    Eigen::VectorXd U(2);
    U << delta, 0.0;

    Eigen::VectorXd X_next(4);
    X_next(0) = v * c_tb * dt_;
    X_next(1) = v * s_tb * dt_;
    X_next(2) = theta + (v / L_) * sb * dt_;
    X_next(3) = v;   // a=0 part

    C = X_next - A * X - B * U;
}

void RDASolver::getObstaclesFromMap() {
    obstacles_.clear();

    if (map_ptr_ == nullptr) {
        LOG(WARNING) << "Map pointer is null. No obstacles added.";
        return;
    }

    const auto& obs_list = map_ptr_->GetObsList();
    if (obs_list.empty()) {
        LOG(INFO) << "No obstacles in the map.";
        return;
    }

    // Since we need to assign each obstacle in obs_list to obstacles_
    for (std::size_t i = 0; i < obs_list.size(); ++i) {
        const auto& obstacle = obs_list[i];

        std::size_t num_edges = obstacle.num_points();
        if (num_edges < 3) {
            LOG(WARNING) << "Obstacle " << i << " has fewer than 3 edges. Skipping.";
            continue;
        }

        // Limit the number of edges for ADMM (we pad with the last edge if fewer,
        // or just use up to max_edge_num_. If num_edges > max_edge_num_, we might need to truncate
        // or we should adapt max_edge_num_. Assuming we use a fixed size for SOCP dimensions).
        Eigen::MatrixXd A_mat = Eigen::MatrixXd::Zero(max_edge_num_, 2);
        Eigen::VectorXd b_vec = Eigen::VectorXd::Zero(max_edge_num_);

        const auto line_segments = obstacle.line_segments();
        const auto center_pts    = obstacle.center();

        std::size_t edge_count = std::min(num_edges, max_edge_num_);

        for (std::size_t j = 0; j < edge_count; ++j) {
            const auto          edge_vector = line_segments[j].unit_direction();
            common::math::Vec2d normal_vector(-edge_vector.y(), edge_vector.x());
            common::math::Vec2d vec_to_center(center_pts.x() - line_segments[j].start().x(),
                                              center_pts.y() - line_segments[j].start().y());
            if (normal_vector.InnerProd(vec_to_center) > 0) { normal_vector *= -1; }
            A_mat(j, 0) = normal_vector.x();
            A_mat(j, 1) = normal_vector.y();
            b_vec(j)    = normal_vector.InnerProd(line_segments[j].start());
        }

        // Pad the remaining rows with the last valid edge to maintain matrix size without changing constraints
        for (std::size_t j = edge_count; j < max_edge_num_; ++j) {
            A_mat(j, 0) = A_mat(edge_count - 1, 0);
            A_mat(j, 1) = A_mat(edge_count - 1, 1);
            b_vec(j)    = b_vec(edge_count - 1);
        }

        RDAObstacle rda_obs;
        // Replicate for all time steps
        for (std::size_t t = 0; t <= T_; ++t) {
            rda_obs.A_list.push_back(A_mat);
            rda_obs.b_list.push_back(b_vec);
        }

        obstacles_.push_back(rda_obs);
    }
}

bool RDASolver::solveSU(const vehicle_model::VehiclePose& start_pose, const vehicle_model::VehiclePose& goal_pose) {
    std::size_t n_var  = 4 * (T_ + 1) + 2 * T_ + T_;
    std::size_t n_dyn  = 4 * T_;
    std::size_t n_cons = n_dyn + n_var;

    std::vector<Eigen::Triplet<double>> H_triplets;
    Eigen::VectorXd                     g = Eigen::VectorXd::Zero(n_var);

    std::vector<Eigen::Triplet<double>> A_triplets;
    Eigen::VectorXd                     lower_bound = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd                     upper_bound = Eigen::VectorXd::Zero(n_cons);

    auto idx_X = [&](std::size_t t) { return 4 * t; };
    auto idx_U = [&](std::size_t t) { return 4 * (T_ + 1) + 2 * t; };
    auto idx_d = [&](std::size_t t) { return 4 * (T_ + 1) + 2 * T_ + t; };

    double w_s       = rda_params_.w_s() > 0 ? rda_params_.w_s() : 1.0;
    double w_u       = rda_params_.w_u() > 0 ? rda_params_.w_u() : 1.0;
    double l1_weight = rda_params_.l1_weight() > 0 ? rda_params_.l1_weight() : 8.0;
    double rho1      = rda_params_.penalty_weight() > 0 ? rda_params_.penalty_weight() : 200.0;

    // Cost: Tracking
    for (std::size_t t = 1; t <= T_; ++t) {
        H_triplets.push_back(Eigen::Triplet<double>(idx_X(t) + 0, idx_X(t) + 0, 2.0 * w_s));
        H_triplets.push_back(Eigen::Triplet<double>(idx_X(t) + 1, idx_X(t) + 1, 2.0 * w_s));

        g(idx_X(t) + 0) -= 2.0 * w_s * init_traj_[t].x();
        g(idx_X(t) + 1) -= 2.0 * w_s * init_traj_[t].y();
    }

    // Cost: Control
    for (std::size_t t = 0; t < T_; ++t) {
        H_triplets.push_back(Eigen::Triplet<double>(idx_U(t) + 0, idx_U(t) + 0, 2.0 * w_u));
        H_triplets.push_back(Eigen::Triplet<double>(idx_U(t) + 1, idx_U(t) + 1, 2.0 * w_u));
    }

    // Cost: Slack L1
    for (std::size_t t = 0; t < T_; ++t) { g(idx_d(t)) += l1_weight; }

    // Cost: ADMM residual
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        for (std::size_t t = 0; t < T_; ++t) {
            Eigen::VectorXd lam_t  = duals_[obs_idx].lam.col(t + 1);
            Eigen::VectorXd mu_t   = duals_[obs_idx].mu.col(t + 1);
            double          z_t    = duals_[obs_idx].z(t);
            double          zeta_t = duals_[obs_idx].zeta(t);

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // Q_s = lam_t^T * A_obs (1 x 2)
            Eigen::RowVector2d Q_s = lam_t.transpose() * A_obs;
            double             c_0 = lam_t.dot(b_obs) + mu_t.dot(h_vehicle_) + z_t - zeta_t;

            std::size_t x_idx = idx_X(t + 1) + 0;
            std::size_t y_idx = idx_X(t + 1) + 1;
            std::size_t d_idx = idx_d(t);

            // H components
            H_triplets.push_back(Eigen::Triplet<double>(x_idx, x_idx, rho1 * Q_s(0) * Q_s(0)));
            H_triplets.push_back(Eigen::Triplet<double>(y_idx, y_idx, rho1 * Q_s(1) * Q_s(1)));
            H_triplets.push_back(Eigen::Triplet<double>(d_idx, d_idx, rho1));

            H_triplets.push_back(Eigen::Triplet<double>(x_idx, y_idx, rho1 * Q_s(0) * Q_s(1)));
            H_triplets.push_back(Eigen::Triplet<double>(y_idx, x_idx, rho1 * Q_s(0) * Q_s(1)));

            H_triplets.push_back(Eigen::Triplet<double>(x_idx, d_idx, -rho1 * Q_s(0)));
            H_triplets.push_back(Eigen::Triplet<double>(d_idx, x_idx, -rho1 * Q_s(0)));

            H_triplets.push_back(Eigen::Triplet<double>(y_idx, d_idx, -rho1 * Q_s(1)));
            H_triplets.push_back(Eigen::Triplet<double>(d_idx, y_idx, -rho1 * Q_s(1)));

            // g components
            g(x_idx) += -rho1 * c_0 * Q_s(0);
            g(y_idx) += -rho1 * c_0 * Q_s(1);
            g(d_idx) += rho1 * c_0;
        }
    }

    // Constraints: Dynamics
    for (std::size_t t = 0; t < T_; ++t) {
        Eigen::MatrixXd A_dyn;
        Eigen::MatrixXd B_dyn;
        Eigen::VectorXd C_dyn;

        double v     = state_traj_(3, t);
        double delta = control_traj_(0, t);
        double theta = state_traj_(2, t);

        computeLinearizedDynamics(v, delta, theta, A_dyn, B_dyn, C_dyn);

        std::size_t row_dyn = 4 * t;
        for (int i = 0; i < 4; ++i) {
            A_triplets.push_back(Eigen::Triplet<double>(row_dyn + i, idx_X(t + 1) + i, 1.0));

            for (int j = 0; j < 4; ++j) {
                A_triplets.push_back(Eigen::Triplet<double>(row_dyn + i, idx_X(t) + j, -A_dyn(i, j)));
            }

            for (int j = 0; j < 2; ++j) {
                A_triplets.push_back(Eigen::Triplet<double>(row_dyn + i, idx_U(t) + j, -B_dyn(i, j)));
            }

            lower_bound(row_dyn + i) = C_dyn(i);
            upper_bound(row_dyn + i) = C_dyn(i);
        }
    }

    // Constraints: Bounds
    double max_speed = dynamic_model_ptr_->GetVehicleParam().max_velocity();
    double max_acc   = dynamic_model_ptr_->GetVehicleParam().max_acc();
    double max_steer = dynamic_model_ptr_->GetVehicleParam().max_steer_angle();
    double inf       = 1e20;

    for (std::size_t i = 0; i < n_var; ++i) {
        std::size_t row = n_dyn + i;
        A_triplets.push_back(Eigen::Triplet<double>(row, i, 1.0));
        lower_bound(row) = -inf;
        upper_bound(row) = inf;
    }

    // Set specific bounds
    double tol_xy    = rda_params_.convergence_tolerance() > 0 ? rda_params_.convergence_tolerance() : 0.05;
    double tol_theta = rda_params_.convergence_tolerance() > 0 ? rda_params_.convergence_tolerance() : 0.05;

    for (std::size_t t = 0; t <= T_; ++t) {
        if (t == 0) {
            lower_bound(n_dyn + idx_X(0) + 0) = start_pose.x - tol_xy;
            upper_bound(n_dyn + idx_X(0) + 0) = start_pose.x + tol_xy;
            lower_bound(n_dyn + idx_X(0) + 1) = start_pose.y - tol_xy;
            upper_bound(n_dyn + idx_X(0) + 1) = start_pose.y + tol_xy;
            lower_bound(n_dyn + idx_X(0) + 2) = start_pose.theta - tol_theta;
            upper_bound(n_dyn + idx_X(0) + 2) = start_pose.theta + tol_theta;
            lower_bound(n_dyn + idx_X(0) + 3) = 0.0;
            upper_bound(n_dyn + idx_X(0) + 3) = 0.0;
        } else if (t == T_) {
            lower_bound(n_dyn + idx_X(T_) + 0) = goal_pose.x - tol_xy;
            upper_bound(n_dyn + idx_X(T_) + 0) = goal_pose.x + tol_xy;
            lower_bound(n_dyn + idx_X(T_) + 1) = goal_pose.y - tol_xy;
            upper_bound(n_dyn + idx_X(T_) + 1) = goal_pose.y + tol_xy;
            lower_bound(n_dyn + idx_X(T_) + 2) = goal_pose.theta - tol_theta;
            upper_bound(n_dyn + idx_X(T_) + 2) = goal_pose.theta + tol_theta;
            lower_bound(n_dyn + idx_X(T_) + 3) = 0.0;
            upper_bound(n_dyn + idx_X(T_) + 3) = 0.0;
        } else {
            lower_bound(n_dyn + idx_X(t) + 3) = -max_speed;
            upper_bound(n_dyn + idx_X(t) + 3) = max_speed;
        }
    }

    for (std::size_t t = 0; t < T_; ++t) {
        lower_bound(n_dyn + idx_U(t) + 0) = -max_steer;
        upper_bound(n_dyn + idx_U(t) + 0) = max_steer;
        lower_bound(n_dyn + idx_U(t) + 1) = -max_acc;
        upper_bound(n_dyn + idx_U(t) + 1) = max_acc;

        lower_bound(n_dyn + idx_d(t)) = 0.0;
        upper_bound(n_dyn + idx_d(t)) = inf;
    }

    Eigen::SparseMatrix<double> H(n_var, n_var);
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    Eigen::SparseMatrix<double> A(n_cons, n_var);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    QPSolver qp_solver(false);
    qp_solver.setVariableNums(n_var);
    qp_solver.setConstraintNums(n_cons);

    if (!qp_solver.Solve(H, g, A, lower_bound, upper_bound)) {
        LOG(ERROR) << "QP Solver failed.";
        return false;
    }

    const Eigen::VectorXd* sol = qp_solver.GetResult();
    for (std::size_t t = 0; t <= T_; ++t) {
        state_traj_.col(t) = sol->segment<4>(idx_X(t));
        if (t < T_) {
            control_traj_.col(t) = sol->segment<2>(idx_U(t));
            slack_d_(t)          = (*sol)(idx_d(t));
        }
    }

    return true;
}

bool RDASolver::solveLamMuZ() {
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        // SOCP Formulation for obstacle obs_idx
        std::size_t E     = max_edge_num_;
        std::size_t n_var = T_ * E + T_ * 4 + T_ + 1;   // lambda, mu, z, t

        // Objective c: minimize t
        Eigen::VectorXd c = Eigen::VectorXd::Zero(n_var);
        c(n_var - 1)      = 1.0;

        // No equality constraints
        Eigen::SparseMatrix<double> A_eq(0, n_var);
        Eigen::VectorXd             b_eq(0);

        // Inequality constraints: h - G w \in K
        // Cones:
        // 1. Positive orthant for \lambda >= 0, \mu >= 0, z >= 0
        // 2. SOC for epigraph cost
        // 3. SOC for ||A_{obs}^T \lambda_k|| <= 1

        std::size_t              lp_dims      = T_ * E + T_ * 4 + T_;
        std::size_t              epigraph_dim = 1 + 3 * T_;
        std::vector<std::size_t> soc_dims_vec;
        soc_dims_vec.push_back(epigraph_dim);
        for (std::size_t t = 0; t < T_; ++t) {
            soc_dims_vec.push_back(3);   // ||A^T \lambda|| <= 1
        }

        std::size_t     total_rows = lp_dims + epigraph_dim + 3 * T_;
        Eigen::VectorXd h          = Eigen::VectorXd::Zero(total_rows);

        std::vector<Eigen::Triplet<double>> G_triplets;

        // Indices
        auto get_lam_idx = [&](std::size_t t) { return t * E; };
        auto get_mu_idx  = [&](std::size_t t) { return T_ * E + t * 4; };
        auto get_z_idx   = [&](std::size_t t) { return T_ * E + T_ * 4 + t; };

        // 1. Positive orthant: \lambda, \mu, z >= 0  => -w_i <= 0 => 0 - (-I * w) >= 0 => G = -I, h = 0
        for (std::size_t i = 0; i < lp_dims; ++i) { G_triplets.push_back(Eigen::Triplet<double>(i, i, -1.0)); }

        std::size_t row_idx = lp_dims;

        // 2. Epigraph SOC: || [sqrt(rho1/2)*v1; sqrt(rho2/2)*v2] || <= t
        // For the first element: t. We want h_i - G_i w = t => h_i = 0, G_i for t is -1.
        G_triplets.push_back(Eigen::Triplet<double>(row_idx, n_var - 1, -1.0));
        row_idx++;

        double rho1        = rda_params_.penalty_weight() > 0 ? rda_params_.penalty_weight() : 200.0;
        double rho2        = 1.0;   // from Python kwargs.get('ro2', 1)
        double sqrt_rho1_2 = std::sqrt(rho1 / 2.0);
        double sqrt_rho2_2 = std::sqrt(rho2 / 2.0);

        for (std::size_t t = 0; t < T_; ++t) {
            double          theta = state_traj_(2, t + 1);
            Eigen::Matrix2d Rot;
            Rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
            Eigen::Vector2d trans = state_traj_.col(t + 1).head(2);

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // Component 1: Im_lammu
            // h - G w = sqrt(rho1/2) * ( \lambda^T(A s - b) - \mu^T h - z - d + \zeta )
            h(row_idx) = sqrt_rho1_2 * (-slack_d_(t) + duals_[obs_idx].zeta(t));

            Eigen::RowVectorXd lam_G1 = -sqrt_rho1_2 * (A_obs * trans - b_obs).transpose();
            for (std::size_t e = 0; e < E; ++e) {
                G_triplets.push_back(Eigen::Triplet<double>(row_idx, get_lam_idx(t) + e, lam_G1(e)));
            }

            for (std::size_t j = 0; j < 4; ++j) {
                G_triplets.push_back(Eigen::Triplet<double>(row_idx, get_mu_idx(t) + j, sqrt_rho1_2 * h_vehicle_(j)));
            }

            G_triplets.push_back(Eigen::Triplet<double>(row_idx, get_z_idx(t), sqrt_rho1_2 * 1.0));

            row_idx++;

            // Component 2 & 3: Hm
            // h - G w = sqrt(rho2/2) * ( G_veh^T \mu + (A_obs Rot)^T \lambda + \xi^T )
            Eigen::Vector2d xi_t  = duals_[obs_idx].xi.row(t + 1).transpose();
            h.segment<2>(row_idx) = sqrt_rho2_2 * xi_t;

            Eigen::MatrixXd lam_G23 = -sqrt_rho2_2 * (A_obs * Rot).transpose();   // 2 x E
            for (std::size_t i = 0; i < 2; ++i) {
                for (std::size_t e = 0; e < E; ++e) {
                    G_triplets.push_back(Eigen::Triplet<double>(row_idx + i, get_lam_idx(t) + e, lam_G23(i, e)));
                }
            }

            Eigen::MatrixXd mu_G23 = -sqrt_rho2_2 * G_vehicle_.transpose();   // 2 x 4
            for (std::size_t i = 0; i < 2; ++i) {
                for (std::size_t j = 0; j < 4; ++j) {
                    G_triplets.push_back(Eigen::Triplet<double>(row_idx + i, get_mu_idx(t) + j, mu_G23(i, j)));
                }
            }

            row_idx += 2;
        }

        // 3. Norm constraint SOC: ||A_{obs}^T \lambda_k|| <= 1
        for (std::size_t t = 0; t < T_; ++t) {
            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            h(row_idx)                   = 1.0;
            row_idx++;

            Eigen::MatrixXd lam_G = -A_obs.transpose();   // 2 x E
            for (std::size_t i = 0; i < 2; ++i) {
                for (std::size_t e = 0; e < E; ++e) {
                    G_triplets.push_back(Eigen::Triplet<double>(row_idx + i, get_lam_idx(t) + e, lam_G(i, e)));
                }
            }
            row_idx += 2;
        }

        Eigen::SparseMatrix<double> G(total_rows, n_var);
        G.setFromTriplets(G_triplets.begin(), G_triplets.end());

        Eigen::VectorXi q_dims(soc_dims_vec.size());
        for (std::size_t i = 0; i < soc_dims_vec.size(); ++i) q_dims(i) = soc_dims_vec[i];

        EiCOS::Solver   solver(G, A_eq, c, h, b_eq, q_dims);
        EiCOS::exitcode status = solver.solve();

        if (status == EiCOS::exitcode::optimal || status == EiCOS::exitcode::close_to_optimal) {
            Eigen::VectorXd sol = solver.solution();
            for (std::size_t t = 0; t < T_; ++t) {
                duals_[obs_idx].lam.col(t + 1) = sol.segment(get_lam_idx(t), E);
                duals_[obs_idx].mu.col(t + 1)  = sol.segment(get_mu_idx(t), 4);
                duals_[obs_idx].z(t)           = sol(get_z_idx(t));
            }
            return true;
        } else {
            LOG(ERROR) << "LamMuZ EiCOS solver failed for obstacle " << obs_idx << " with status "
                       << static_cast<int>(status);
            return false;
        }
    };

    return true;
}

void RDASolver::updateMultipliers() {
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        for (std::size_t t = 0; t < T_; ++t) {
            // Retrieve variables
            Eigen::VectorXd lam_t = duals_[obs_idx].lam.col(t + 1);
            Eigen::VectorXd mu_t  = duals_[obs_idx].mu.col(t + 1);
            double          z_t   = duals_[obs_idx].z(t);

            double          theta = state_traj_(2, t + 1);
            Eigen::Matrix2d Rot;
            Rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
            Eigen::Vector2d trans = state_traj_.col(t + 1).head(2);

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // 1. Update xi
            // xi_t = xi_t + G_veh^T * mu_t + (A_obs * Rot)^T * lam_t
            Eigen::Vector2d Hm_val = G_vehicle_.transpose() * mu_t + (A_obs * Rot).transpose() * lam_t;
            duals_[obs_idx].xi.row(t + 1) += Hm_val.transpose();

            // 2. Update zeta
            // zeta_t = zeta_t + lam_t^T * (A_obs * trans - b_obs) - mu_t^T * h_veh - d_t - z_t
            double Im_val = lam_t.dot(A_obs * trans - b_obs) - mu_t.dot(h_vehicle_);
            duals_[obs_idx].zeta(t) += Im_val - slack_d_(t) - z_t;
        }
    }
}

bool RDASolver::Process(const vehicle_model::sdv_path&               init_path,
                        std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                        std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    if (init_path.empty()) return false;
    T_ = init_path.size() - 1;

    state_traj_.resize(4, T_ + 1);
    control_traj_.resize(2, T_);
    slack_d_.resize(T_);

    init_traj_.clear();
    opt_traj_.clear();

    // Initialize state and control trajectory
    for (std::size_t i = 0; i <= T_; ++i) {
        state_traj_(0, i) = init_path[i].x;
        state_traj_(1, i) = init_path[i].y;
        state_traj_(2, i) = init_path[i].theta;
        state_traj_(3, i) = 0.0;

        init_traj_.push_back(Eigen::Vector4d(init_path[i].x, init_path[i].y, init_path[i].theta, 0.0));

        if (i < T_) {
            control_traj_(0, i) = 0.0;
            control_traj_(1, i) = 0.0;
            slack_d_(i)         = 0.1;
        }
    }

    getObstaclesFromMap();

    // Initialize dual variables
    duals_.resize(obstacles_.size());
    for (auto& dual : duals_) {
        dual.lam.setZero(max_edge_num_, T_ + 1);
        dual.mu.setZero(4, T_ + 1);
        dual.z.setZero(T_);
        dual.xi.setZero(T_ + 1, 2);
        dual.zeta.setZero(T_);
    }

    if (start_pose_ptr == nullptr || goal_pose_ptr == nullptr) {
        LOG(ERROR) << "Start or goal pose is null";
        return false;
    }

    int max_iter = rda_params_.max_iter() > 0 ? rda_params_.max_iter() : 5;
    for (int iter = 0; iter < max_iter; ++iter) {
        if (!solveSU(*start_pose_ptr, *goal_pose_ptr)) {
            LOG(ERROR) << "Failed to solve SU subproblem";
            return false;
        }
        if (!solveLamMuZ()) {
            LOG(ERROR) << "Failed to solve LamMuZ subproblem";
            return false;
        }
        updateMultipliers();
        LOG(INFO) << "RDA iteration " << iter + 1 << " completed.";
    }

    opt_traj_.clear();
    for (std::size_t i = 0; i <= T_; ++i) {
        opt_traj_.push_back(
            Eigen::Vector4d(state_traj_(0, i), state_traj_(1, i), state_traj_(2, i), state_traj_(3, i)));
    }

    return true;
}

}   // namespace backend
}   // namespace planning
