/*
 * @Author: wenqing-2021 yuansj@hnu.edu.cn
 * @FilePath: /DrivingTrajectoryPlanning/src/planner/rda/rda_planner.cpp
 * @Description: RDA (Reduced Dual-space ADMM) planner, C++ implementation.
 *               Reference: https://github.com/hanruihua/RDA-planner/blob/main/RDA_planner/rda_solver.py
 *               Differences from the Python version:
 *                 1) Start and goal pose constraints are added.
 *                 2) Kinematic bicycle model without slip angle beta; control = (steer angle, acceleration).
 *                 3) Horizon T is determined by the input path; per-segment time interval dt is an
 *                    optimization variable (bilinear dynamics are linearized at the current iterate).
 *                 4) The SU subproblem is solved as a QP (OSQP) instead of an SOCP.
 */
#include "rda_planner.h"
#include "logger.h"
#include <cmath>

namespace planning {
namespace backend {

namespace {
// Use a moderate "infinity" for OSQP bounds: 1e20 degrades the constraint
// matrix conditioning and can make OSQP report spurious primal infeasibility.
constexpr double kInf = 1e10;
// Trust region (rad) limiting per-iteration change of theta so that the
// first-order linearization of R(theta) stays valid (large-heading parking).
constexpr double kThetaTrustRegion = 0.5;
}   // namespace

RDASolver::RDASolver(const std::shared_ptr<vehicle_model::KinematicModel>& dynamic_model_ptr,
                     const std::shared_ptr<map::Map>& map_ptr, const params::RDAParams& rda_params, double dt)
    : dynamic_model_ptr_(dynamic_model_ptr)
    , map_ptr_(map_ptr)
    , rda_params_(rda_params)
    , dt_(dt) {

    L_ = dynamic_model_ptr_->GetVehicleParam().wheel_base();

    // Vehicle bounding box constraints: G * c <= h  (4 edges, axis-aligned box in body frame)
    G_vehicle_.resize(4, 2);
    G_vehicle_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;

    double half_length = dynamic_model_ptr_->GetVehicleParam().length() / 2.0;
    double half_width  = dynamic_model_ptr_->GetVehicleParam().width() / 2.0;

    h_vehicle_.resize(4);
    h_vehicle_ << half_length, half_width, half_length, half_width;
}

// ---------------------------------------------------------------------------
// Linearized kinematic bicycle model (no slip angle beta)
//   x'     = x + v * cos(theta) * dt
//   y'     = y + v * sin(theta) * dt
//   theta' = theta + (v / L) * tan(delta) * dt
//   v'     = v + a * dt
// Linearized at (v, delta, theta, a, dt):
//   X_{t+1} ~= A * X_t + B * U_t + D * dt_t + C
// ---------------------------------------------------------------------------
void RDASolver::computeLinearizedDynamics(double v, double delta, double theta, double a, double dt, Eigen::MatrixXd& A,
                                          Eigen::MatrixXd& B, Eigen::VectorXd& D, Eigen::VectorXd& C) {
    double c_t   = std::cos(theta);
    double s_t   = std::sin(theta);
    double tan_d = std::tan(delta);
    double sec2  = 1.0 / (std::cos(delta) * std::cos(delta));

    // A = df/dX
    A.setIdentity(4, 4);
    A(0, 2) = -v * s_t * dt;
    A(0, 3) = c_t * dt;
    A(1, 2) = v * c_t * dt;
    A(1, 3) = s_t * dt;
    A(2, 3) = (1.0 / L_) * tan_d * dt;

    // B = df/dU
    B.setZero(4, 2);
    B(2, 0) = (v / L_) * sec2 * dt;
    B(3, 1) = dt;

    // D = df/d(dt)  (time interval is an optimization variable)
    D.resize(4);
    D << v * c_t, v * s_t, (v / L_) * tan_d, a;

    // C = f(X_ref, U_ref, dt_ref) - A * X_ref - B * U_ref - D * dt_ref
    Eigen::VectorXd X_ref(4);
    X_ref << 0.0, 0.0, theta, v;
    Eigen::VectorXd U_ref(2);
    U_ref << delta, a;

    Eigen::VectorXd f_ref(4);
    f_ref << v * c_t * dt, v * s_t * dt, theta + (v / L_) * tan_d * dt, v + a * dt;

    C = f_ref - A * X_ref - B * U_ref - D * dt;
}

// ---------------------------------------------------------------------------
// Convert map obstacles into per-timestep halfspace representations.
// Each obstacle polygon with edge_num edges gives A (edge_num x 2), b (edge_num x 1)
// such that A * p <= b describes the obstacle interior (normals point outward).
// ---------------------------------------------------------------------------
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

    // All obstacles are considered. (A previous distance-based filter was removed:
    // it was not principled and could drop obstacles that the vehicle must avoid.)
    for (std::size_t i = 0; i < obs_list.size(); ++i) {
        const auto& obstacle = obs_list[i];

        std::size_t num_edges = obstacle.num_points();
        if (num_edges < 3) {
            LOG(WARNING) << "Obstacle " << i << " has fewer than 3 edges. Skipping.";
            continue;
        }

        Eigen::MatrixXd A_mat = Eigen::MatrixXd::Zero(num_edges, 2);
        Eigen::VectorXd b_vec = Eigen::VectorXd::Zero(num_edges);

        const auto line_segments = obstacle.line_segments();
        const auto center_pts    = obstacle.center();

        for (std::size_t j = 0; j < num_edges; ++j) {
            const auto          edge_vector = line_segments[j].unit_direction();
            common::math::Vec2d normal_vector(-edge_vector.y(), edge_vector.x());
            common::math::Vec2d vec_to_center(center_pts.x() - line_segments[j].start().x(),
                                              center_pts.y() - line_segments[j].start().y());
            // Ensure the normal points outward (away from the polygon center)
            if (normal_vector.InnerProd(vec_to_center) > 0) { normal_vector *= -1; }
            A_mat(j, 0) = normal_vector.x();
            A_mat(j, 1) = normal_vector.y();
            b_vec(j)    = normal_vector.InnerProd(line_segments[j].start());
        }

        RDAObstacle rda_obs;
        rda_obs.edge_num = num_edges;
        // Replicate for all time steps (static obstacles)
        for (std::size_t t = 0; t <= T_; ++t) {
            rda_obs.A_list.push_back(A_mat);
            rda_obs.b_list.push_back(b_vec);
        }

        obstacles_.push_back(rda_obs);
    }
}

// ---------------------------------------------------------------------------
// SU subproblem: QP over [X(4(T+1)), U(2T), d(T), dt(T)]
//   minimize  tracking + control + slack + ADMM residuals + dt regularization
//   s.t.    linearized dynamics (with dt), start/goal box constraints,
//           state/control/slack/dt bounds
// ---------------------------------------------------------------------------
bool RDASolver::solveSU(const vehicle_model::VehiclePose& start_pose, const vehicle_model::VehiclePose& goal_pose) {
    // Variable layout:
    //   X:  4*(T+1)          [x, y, theta, v] per step
    //   U:  2*T              [delta, a] per step
    //   d:  T                safety-distance slack per step
    //   dt: T                time interval per step
    //   r:  n_obs*T          auxiliary variable r >= max(0, -Im) (accelerated ADMM)
    const std::size_t n_obs = obstacles_.size();
    const std::size_t n_x   = 4 * (T_ + 1);
    const std::size_t n_u   = 2 * T_;
    const std::size_t n_d   = T_;
    const std::size_t n_dt  = T_;
    const std::size_t n_r   = n_obs * T_;
    const std::size_t n_var = n_x + n_u + n_d + n_dt + n_r;

    const std::size_t n_dyn  = 4 * T_;                   // dynamics equality constraints
    const std::size_t n_rcon = n_obs * T_;               // r >= -Im  inequality constraints
    const std::size_t n_cons = n_dyn + n_rcon + n_var;   // dynamics + r-constraints + variable bounds

    auto idx_X  = [&](std::size_t t) { return 4 * t; };
    auto idx_U  = [&](std::size_t t) { return n_x + 2 * t; };
    auto idx_d  = [&](std::size_t t) { return n_x + n_u + t; };
    auto idx_dt = [&](std::size_t t) { return n_x + n_u + n_d + t; };
    auto idx_r  = [&](std::size_t o, std::size_t t) { return n_x + n_u + n_d + n_dt + o * T_ + t; };

    // Parameters (with defaults matching the Python implementation)
    const double w_s        = rda_params_.w_s() > 0 ? rda_params_.w_s() : 1.0;
    const double w_u        = rda_params_.w_u() > 0 ? rda_params_.w_u() : 1.0;
    const double slack_gain = rda_params_.l1_weight() > 0 ? rda_params_.l1_weight() : 8.0;
    const double rho1       = rda_params_.penalty_weight() > 0 ? rda_params_.penalty_weight() : 200.0;
    const double rho2       = rda_params_.ro2() > 0 ? rda_params_.ro2() : 1.0;
    const double w_dt       = rda_params_.w_dt() > 0 ? rda_params_.w_dt() : 1.0;
    const double min_sd     = rda_params_.min_sd() > 0 ? rda_params_.min_sd() : 0.1;
    const double max_sd     = rda_params_.max_sd() > 0 ? rda_params_.max_sd() : 1.0;
    const double dt_min     = rda_params_.dt_min() > 0 ? rda_params_.dt_min() : 0.5 * dt_;
    const double dt_max     = rda_params_.dt_max() > 0 ? rda_params_.dt_max() : 2.0 * dt_;

    std::vector<Eigen::Triplet<double>> H_triplets;
    Eigen::VectorXd                     g = Eigen::VectorXd::Zero(n_var);

    std::vector<Eigen::Triplet<double>> A_triplets;
    Eigen::VectorXd                     lower_bound = Eigen::VectorXd::Zero(n_cons);
    Eigen::VectorXd                     upper_bound = Eigen::VectorXd::Zero(n_cons);

    // ---- Cost: path tracking (x, y, theta) --------------------------------
    for (std::size_t t = 1; t <= T_; ++t) {
        H_triplets.emplace_back(idx_X(t) + 0, idx_X(t) + 0, 2.0 * w_s);
        H_triplets.emplace_back(idx_X(t) + 1, idx_X(t) + 1, 2.0 * w_s);
        H_triplets.emplace_back(idx_X(t) + 2, idx_X(t) + 2, 2.0 * w_s);
        g(idx_X(t) + 0) -= 2.0 * w_s * init_traj_[t].x();
        g(idx_X(t) + 1) -= 2.0 * w_s * init_traj_[t].y();
        g(idx_X(t) + 2) -= 2.0 * w_s * init_traj_[t].z();
    }

    // ---- Cost: control magnitude -------------------------------------------
    for (std::size_t t = 0; t < T_; ++t) {
        H_triplets.emplace_back(idx_U(t) + 0, idx_U(t) + 0, 2.0 * w_u);
        H_triplets.emplace_back(idx_U(t) + 1, idx_U(t) + 1, 2.0 * w_u);
    }

    // ---- Cost: safety-distance slack (maximize d => minimize -slack_gain * d)
    for (std::size_t t = 0; t < T_; ++t) { g(idx_d(t)) -= slack_gain; }

    // ---- Cost: dt regularization toward nominal dt --------------------------
    for (std::size_t t = 0; t < T_; ++t) {
        H_triplets.emplace_back(idx_dt(t), idx_dt(t), 2.0 * w_dt);
        g(idx_dt(t)) -= 2.0 * w_dt * dt_;
    }

    // ---- Cost: ADMM residuals (Im and Hm) -----------------------------------
    // Im_t = lam^T (A_obs * trans - b_obs) - mu^T h_veh - d_t - z_t + zeta_t
    //      = Q_s * trans - c_0 - d_t          (linear in x, y, d)
    //   where Q_s = lam^T A_obs (1x2), c_0 = lam^T b + mu^T h + z - zeta
    // Accelerated ADMM penalizes only the violated part: 0.5*rho1*||neg(Im)||^2.
    // This is handled with an auxiliary variable r >= max(0, -Im):
    //   minimize 0.5*rho1*r^2   s.t.  r >= -Im  (i.e. -Im - r <= 0), r >= 0.
    //
    // Hm_t = mu^T G_veh + lam^T A_obs R(theta_t) + xi_t   (1x2)
    // R(theta) linearized at reference theta_ref:
    //   R(theta) ~= R_ref + R'_ref * (theta - theta_ref)
    // Hm is linear in theta. Penalty: 0.5 * rho2 * ||Hm||^2 -> quadratic in theta.
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        for (std::size_t t = 0; t < T_; ++t) {
            const Eigen::VectorXd& lam_t  = duals_[obs_idx].lam.col(t + 1);
            const Eigen::VectorXd& mu_t   = duals_[obs_idx].mu.col(t + 1);
            double                 z_t    = duals_[obs_idx].z(t);
            double                 zeta_t = duals_[obs_idx].zeta(t);
            const Eigen::Vector2d  xi_t   = duals_[obs_idx].xi.row(t + 1).transpose();

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // --- Im residual via auxiliary variable r (accelerated) ---
            Eigen::RowVector2d Q_s = lam_t.transpose() * A_obs;   // 1 x 2
            double             c_0 = lam_t.dot(b_obs) + mu_t.dot(h_vehicle_) + z_t - zeta_t;

            std::size_t x_idx = idx_X(t + 1) + 0;
            std::size_t y_idx = idx_X(t + 1) + 1;
            std::size_t d_idx = idx_d(t);
            std::size_t r_idx = idx_r(obs_idx, t);

            // Penalty 0.5*rho1*r^2
            H_triplets.emplace_back(r_idx, r_idx, rho1);

            // Constraint r >= -Im  <=>  -(Q_s*x + Q_s*y - c_0 - d) - r <= 0
            //   => -Q_s(0)*x - Q_s(1)*y + d - r <= -c_0
            std::size_t row_r = n_dyn + obs_idx * T_ + t;
            A_triplets.emplace_back(row_r, x_idx, -Q_s(0));
            A_triplets.emplace_back(row_r, y_idx, -Q_s(1));
            A_triplets.emplace_back(row_r, d_idx, 1.0);
            A_triplets.emplace_back(row_r, r_idx, -1.0);
            lower_bound(row_r) = -kInf;
            upper_bound(row_r) = -c_0;

            // --- Hm residual (involves theta) ---
            // Hm = mu^T G + lam^T A R(theta) + xi  (1x2 row vector)
            // R(theta) = [[c, -s],[s, c]], linearized at theta_ref = state_traj_(2, t+1)
            double theta_ref = state_traj_(2, t + 1);
            double c_r       = std::cos(theta_ref);
            double s_r       = std::sin(theta_ref);

            // lam^T A R = p^T R where p = A^T lam (2x1)
            Eigen::Vector2d p = A_obs.transpose() * lam_t;
            // p^T R(theta) = [p0*c + p1*s, -p0*s + p1*c]
            // d/d(theta) of p^T R = [-p0*s + p1*c, -p0*c - p1*s]
            double h0_const = mu_t.dot(G_vehicle_.col(0)) + p(0) * c_r + p(1) * s_r + xi_t(0);
            double h1_const = mu_t.dot(G_vehicle_.col(1)) - p(0) * s_r + p(1) * c_r + xi_t(1);
            double h0_deriv = -p(0) * s_r + p(1) * c_r;
            double h1_deriv = -p(0) * c_r - p(1) * s_r;

            // 0.5*rho2 * sum_j (h_const_j + h_deriv_j*(theta - theta_ref))^2
            // term_j = h_deriv_j * theta + (h_const_j - h_deriv_j * theta_ref)
            std::size_t th_idx = idx_X(t + 1) + 2;
            double      dd0    = h0_deriv;
            double      dd1    = h1_deriv;
            double      cc0    = h0_const - h0_deriv * theta_ref;
            double      cc1    = h1_const - h1_deriv * theta_ref;

            H_triplets.emplace_back(th_idx, th_idx, rho2 * (dd0 * dd0 + dd1 * dd1));
            g(th_idx) += rho2 * (dd0 * cc0 + dd1 * cc1);
        }
    }

    // ---- Constraints: linearized dynamics (with dt variable) -----------------
    // X_{t+1} = A X_t + B U_t + D dt_t + C
    // => X_{t+1} - A X_t - B U_t - D dt_t = C
    for (std::size_t t = 0; t < T_; ++t) {
        Eigen::MatrixXd A_dyn, B_dyn;
        Eigen::VectorXd D_dyn, C_dyn;

        double v     = state_traj_(3, t);
        double delta = control_traj_(0, t);
        double theta = state_traj_(2, t);
        double a     = control_traj_(1, t);
        double dt_t  = dt_traj_(t);

        computeLinearizedDynamics(v, delta, theta, a, dt_t, A_dyn, B_dyn, D_dyn, C_dyn);

        std::size_t row = 4 * t;
        for (int i = 0; i < 4; ++i) {
            A_triplets.emplace_back(row + i, idx_X(t + 1) + i, 1.0);
            for (int j = 0; j < 4; ++j) { A_triplets.emplace_back(row + i, idx_X(t) + j, -A_dyn(i, j)); }
            for (int j = 0; j < 2; ++j) { A_triplets.emplace_back(row + i, idx_U(t) + j, -B_dyn(i, j)); }
            A_triplets.emplace_back(row + i, idx_dt(t), -D_dyn(i));

            lower_bound(row + i) = C_dyn(i);
            upper_bound(row + i) = C_dyn(i);
        }
    }

    // ---- Constraints: variable bounds ----------------------------------------
    // Variable-bound rows start after the dynamics and r-constraint rows.
    const std::size_t bnd = n_dyn + n_rcon;

    double max_speed = dynamic_model_ptr_->GetVehicleParam().max_velocity();
    double max_acc   = dynamic_model_ptr_->GetVehicleParam().max_acc();
    double max_steer = dynamic_model_ptr_->GetVehicleParam().max_steer_angle();

    // Default: all variables unbounded
    for (std::size_t i = 0; i < n_var; ++i) {
        std::size_t row = bnd + i;
        A_triplets.emplace_back(row, i, 1.0);
        lower_bound(row) = -kInf;
        upper_bound(row) = kInf;
    }

    // Start / goal box constraints (with tolerance)
    double tol_xy    = rda_params_.convergence_tolerance() > 0 ? rda_params_.convergence_tolerance() : 0.05;
    double tol_theta = tol_xy;

    for (std::size_t t = 0; t <= T_; ++t) {
        if (t == 0) {
            lower_bound(bnd + idx_X(0) + 0) = start_pose.x - tol_xy;
            upper_bound(bnd + idx_X(0) + 0) = start_pose.x + tol_xy;
            lower_bound(bnd + idx_X(0) + 1) = start_pose.y - tol_xy;
            upper_bound(bnd + idx_X(0) + 1) = start_pose.y + tol_xy;
            lower_bound(bnd + idx_X(0) + 2) = start_pose.theta - tol_theta;
            upper_bound(bnd + idx_X(0) + 2) = start_pose.theta + tol_theta;
            lower_bound(bnd + idx_X(0) + 3) = 0.0;
            upper_bound(bnd + idx_X(0) + 3) = 0.0;
        } else if (t == T_) {
            lower_bound(bnd + idx_X(T_) + 0) = goal_pose.x - tol_xy;
            upper_bound(bnd + idx_X(T_) + 0) = goal_pose.x + tol_xy;
            lower_bound(bnd + idx_X(T_) + 1) = goal_pose.y - tol_xy;
            upper_bound(bnd + idx_X(T_) + 1) = goal_pose.y + tol_xy;
            lower_bound(bnd + idx_X(T_) + 2) = goal_pose.theta - tol_theta;
            upper_bound(bnd + idx_X(T_) + 2) = goal_pose.theta + tol_theta;
            lower_bound(bnd + idx_X(T_) + 3) = 0.0;
            upper_bound(bnd + idx_X(T_) + 3) = 0.0;
        } else {
            lower_bound(bnd + idx_X(t) + 3) = -max_speed;
            upper_bound(bnd + idx_X(t) + 3) = max_speed;

            // Trust region on theta: keep the linearization of R(theta) valid by
            // limiting how far theta can move from the current reference in one
            // ADMM iteration. Skip the trust region on the very first iteration
            // (reference is a coarse straight-line guess that may be far from the
            // feasible manifold); apply it from the second iteration onward.
            if (admm_iter_ > 0) {
                double theta_ref                = state_traj_(2, t);
                lower_bound(bnd + idx_X(t) + 2) = theta_ref - kThetaTrustRegion;
                upper_bound(bnd + idx_X(t) + 2) = theta_ref + kThetaTrustRegion;
            }
        }
    }

    for (std::size_t t = 0; t < T_; ++t) {
        // Control bounds
        lower_bound(bnd + idx_U(t) + 0) = -max_steer;
        upper_bound(bnd + idx_U(t) + 0) = max_steer;
        lower_bound(bnd + idx_U(t) + 1) = -max_acc;
        upper_bound(bnd + idx_U(t) + 1) = max_acc;
        // Safety-distance slack bounds
        lower_bound(bnd + idx_d(t)) = min_sd;
        upper_bound(bnd + idx_d(t)) = max_sd;
        // Time interval bounds
        lower_bound(bnd + idx_dt(t)) = dt_min;
        upper_bound(bnd + idx_dt(t)) = dt_max;
    }

    // Auxiliary variable r >= 0
    for (std::size_t i = 0; i < n_r; ++i) {
        lower_bound(bnd + n_x + n_u + n_d + n_dt + i) = 0.0;
        upper_bound(bnd + n_x + n_u + n_d + n_dt + i) = kInf;
    }

    // ---- Assemble and solve ---------------------------------------------------
    Eigen::SparseMatrix<double> H(n_var, n_var);
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    Eigen::SparseMatrix<double> A_mat(n_cons, n_var);
    A_mat.setFromTriplets(A_triplets.begin(), A_triplets.end());

    QPSolver qp_solver(false);
    qp_solver.setVariableNums(n_var);
    qp_solver.setConstraintNums(n_cons);

    if (!qp_solver.Solve(H, g, A_mat, lower_bound, upper_bound)) {
        LOG(ERROR) << "SU QP solver failed.";
        return false;
    }

    const Eigen::VectorXd* sol = qp_solver.GetResult();

    // Health check: OSQP may report "solved" but return a huge/NaN solution when
    // the QP is primal infeasible (e.g. an over-tight trust region). Reject such
    // solutions instead of poisoning the trajectory.
    if (sol->hasNaN() || sol->cwiseAbs().maxCoeff() > 1e6) {
        LOG(ERROR) << "SU QP returned an invalid solution (max|x|=" << sol->cwiseAbs().maxCoeff()
                   << "); likely primal infeasible.";
        return false;
    }

    for (std::size_t t = 0; t <= T_; ++t) {
        state_traj_.col(t) = sol->segment<4>(idx_X(t));
        if (t < T_) {
            control_traj_.col(t) = sol->segment<2>(idx_U(t));
            slack_d_(t)          = (*sol)(idx_d(t));
            dt_traj_(t)          = (*sol)(idx_dt(t));
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// LamMuZ subproblem: one SOCP per obstacle, solved with EiCOS.
// Variables: [lam(E*T), mu(4*T), z(T), t_epigraph(1)]
//   minimize  t
//   s.t.      || [sqrt(rho1/2)*Im_t; sqrt(rho2/2)*Hm_t] || <= t   (epigraph SOC)
//             || A_obs^T lam_t || <= 1                            (dual-norm SOC)
//             lam >= 0, mu >= 0, z >= 0                           (positive orthant)
// ---------------------------------------------------------------------------
bool RDASolver::solveLamMuZ() {
    const double rho1        = rda_params_.penalty_weight() > 0 ? rda_params_.penalty_weight() : 200.0;
    const double rho2        = rda_params_.ro2() > 0 ? rda_params_.ro2() : 1.0;
    const double sqrt_rho1_2 = std::sqrt(rho1 / 2.0);
    const double sqrt_rho2_2 = std::sqrt(rho2 / 2.0);

    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        const std::size_t E     = obstacles_[obs_idx].edge_num;
        const std::size_t n_var = T_ * E + T_ * 4 + T_ + 1;   // lam, mu, z, t_epigraph

        // Objective: minimize t_epigraph
        Eigen::VectorXd c = Eigen::VectorXd::Zero(n_var);
        c(n_var - 1)      = 1.0;

        // No equality constraints
        Eigen::SparseMatrix<double> A_eq(0, n_var);
        Eigen::VectorXd             b_eq(0);

        // Cone layout: [positive orthant (lam, mu, z)] [epigraph SOC] [norm SOCs]
        const std::size_t lp_dims      = T_ * E + T_ * 4 + T_;
        const std::size_t epigraph_dim = 1 + 3 * T_;   // t + (Im, Hm0, Hm1) per step

        std::vector<std::size_t> soc_dims_vec;
        soc_dims_vec.push_back(epigraph_dim);
        for (std::size_t t = 0; t < T_; ++t) { soc_dims_vec.push_back(3); }

        const std::size_t total_rows = lp_dims + epigraph_dim + 3 * T_;
        Eigen::VectorXd   h          = Eigen::VectorXd::Zero(total_rows);

        std::vector<Eigen::Triplet<double>> G_triplets;

        auto lam_idx = [&](std::size_t t) { return t * E; };
        auto mu_idx  = [&](std::size_t t) { return T_ * E + t * 4; };
        auto z_idx   = [&](std::size_t t) { return T_ * E + T_ * 4 + t; };

        // 1) Positive orthant: lam, mu, z >= 0  =>  -I * w <= 0
        for (std::size_t i = 0; i < lp_dims; ++i) { G_triplets.emplace_back(i, i, -1.0); }

        std::size_t row = lp_dims;

        // 2) Epigraph SOC: first element is t_epigraph
        G_triplets.emplace_back(row, n_var - 1, -1.0);
        row++;

        for (std::size_t t = 0; t < T_; ++t) {
            double          theta = state_traj_(2, t + 1);
            Eigen::Matrix2d Rot;
            Rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
            Eigen::Vector2d trans = state_traj_.col(t + 1).head(2);

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // --- Im component: sqrt(rho1/2) * (lam^T(A s - b) - mu^T h - z - d + zeta) ---
            h(row) = sqrt_rho1_2 * (-slack_d_(t) + duals_[obs_idx].zeta(t));

            Eigen::RowVectorXd lam_G1 = -sqrt_rho1_2 * (A_obs * trans - b_obs).transpose();
            for (std::size_t e = 0; e < E; ++e) { G_triplets.emplace_back(row, lam_idx(t) + e, lam_G1(e)); }
            for (std::size_t j = 0; j < 4; ++j) {
                G_triplets.emplace_back(row, mu_idx(t) + j, sqrt_rho1_2 * h_vehicle_(j));
            }
            G_triplets.emplace_back(row, z_idx(t), sqrt_rho1_2);
            row++;

            // --- Hm components: sqrt(rho2/2) * (G^T mu + (A R)^T lam + xi) ---
            Eigen::Vector2d xi_t = duals_[obs_idx].xi.row(t + 1).transpose();
            h.segment<2>(row)    = sqrt_rho2_2 * xi_t;

            Eigen::MatrixXd lam_G23 = -sqrt_rho2_2 * (A_obs * Rot).transpose();   // 2 x E
            for (int i = 0; i < 2; ++i) {
                for (std::size_t e = 0; e < E; ++e) { G_triplets.emplace_back(row + i, lam_idx(t) + e, lam_G23(i, e)); }
            }
            Eigen::MatrixXd mu_G23 = -sqrt_rho2_2 * G_vehicle_.transpose();   // 2 x 4
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 4; ++j) { G_triplets.emplace_back(row + i, mu_idx(t) + j, mu_G23(i, j)); }
            }
            row += 2;
        }

        // 3) Dual-norm SOC: || A_obs^T lam_t || <= 1
        for (std::size_t t = 0; t < T_; ++t) {
            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            h(row)                       = 1.0;
            row++;
            Eigen::MatrixXd lam_G = -A_obs.transpose();   // 2 x E
            for (int i = 0; i < 2; ++i) {
                for (std::size_t e = 0; e < E; ++e) { G_triplets.emplace_back(row + i, lam_idx(t) + e, lam_G(i, e)); }
            }
            row += 2;
        }

        Eigen::SparseMatrix<double> G(total_rows, n_var);
        G.setFromTriplets(G_triplets.begin(), G_triplets.end());

        Eigen::VectorXi q_dims(soc_dims_vec.size());
        for (std::size_t i = 0; i < soc_dims_vec.size(); ++i) { q_dims(i) = soc_dims_vec[i]; }

        EiCOS::Solver   solver(G, A_eq, c, h, b_eq, q_dims);
        EiCOS::exitcode status = solver.solve();

        bool ok = (status == EiCOS::exitcode::optimal || status == EiCOS::exitcode::close_to_optimal ||
                   status == EiCOS::exitcode::close_to_primal_infeasible);
        if (ok) {
            const Eigen::VectorXd& sol = solver.solution();
            for (std::size_t t = 0; t < T_; ++t) {
                duals_[obs_idx].lam.col(t + 1) = sol.segment(lam_idx(t), E);
                duals_[obs_idx].mu.col(t + 1)  = sol.segment(mu_idx(t), 4);
                duals_[obs_idx].z(t)           = sol(z_idx(t));
            }
            LOG(INFO) << "LamMuZ obs " << obs_idx << ": |lam|=" << duals_[obs_idx].lam.norm()
                      << " |mu|=" << duals_[obs_idx].mu.norm() << " |z|=" << duals_[obs_idx].z.norm();
        } else {
            // Keep the previous-iteration dual variables for this obstacle and
            // continue with the others instead of aborting the whole ADMM solve.
            LOG(WARNING) << "LamMuZ EiCOS failed for obstacle " << obs_idx << " (status " << static_cast<int>(status)
                         << "); keeping previous duals.";
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Dual variable update (xi and zeta)
//   xi_t   += G_veh^T mu_t + (A_obs R)^T lam_t
//   zeta_t += lam_t^T (A_obs trans - b_obs) - mu_t^T h_veh - d_t - z_t
// ---------------------------------------------------------------------------
void RDASolver::updateMultipliers() {
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        for (std::size_t t = 0; t < T_; ++t) {
            const Eigen::VectorXd& lam_t = duals_[obs_idx].lam.col(t + 1);
            const Eigen::VectorXd& mu_t  = duals_[obs_idx].mu.col(t + 1);
            double                 z_t   = duals_[obs_idx].z(t);

            double          theta = state_traj_(2, t + 1);
            Eigen::Matrix2d Rot;
            Rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
            Eigen::Vector2d trans = state_traj_.col(t + 1).head(2);

            const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
            const Eigen::VectorXd& b_obs = obstacles_[obs_idx].b_list[t + 1];

            // Update xi
            Eigen::Vector2d Hm_val = G_vehicle_.transpose() * mu_t + (A_obs * Rot).transpose() * lam_t;
            duals_[obs_idx].xi.row(t + 1) += Hm_val.transpose();

            // Update zeta
            double Im_val = lam_t.dot(A_obs * trans - b_obs) - mu_t.dot(h_vehicle_);
            duals_[obs_idx].zeta(t) += Im_val - slack_d_(t) - z_t;
        }
    }
}

// ---------------------------------------------------------------------------
// Main entry: ADMM iteration
// ---------------------------------------------------------------------------
bool RDASolver::Process(const vehicle_model::sdv_path&               init_path,
                        std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                        std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    if (init_path.empty()) { return false; }
    T_ = init_path.size() - 1;

    state_traj_.resize(4, T_ + 1);
    control_traj_.resize(2, T_);
    slack_d_.resize(T_);
    dt_traj_.resize(T_);

    init_traj_.clear();
    opt_traj_.clear();

    // Estimate a nominal speed from the path length so that the linearized
    // dynamics are non-degenerate at the first ADMM iteration (v = 0 would make
    // x_{t+1} = x_t and conflict with the start/goal constraints).
    double path_length = 0.0;
    for (std::size_t i = 1; i <= T_; ++i) {
        double dx = init_path[i].x - init_path[i - 1].x;
        double dy = init_path[i].y - init_path[i - 1].y;
        path_length += std::hypot(dx, dy);
    }
    double total_time = T_ * dt_;
    double v_nominal  = (total_time > 1e-6) ? path_length / total_time : 0.0;
    double max_speed  = dynamic_model_ptr_->GetVehicleParam().max_velocity();
    v_nominal         = std::min(v_nominal, 0.8 * max_speed);

    // Initialize state / control / slack / dt from the input path
    for (std::size_t i = 0; i <= T_; ++i) {
        state_traj_(0, i) = init_path[i].x;
        state_traj_(1, i) = init_path[i].y;
        state_traj_(2, i) = init_path[i].theta;
        // Give interior points a nominal speed; endpoints stay at zero.
        state_traj_(3, i) = (i == 0 || i == T_) ? 0.0 : v_nominal;

        init_traj_.emplace_back(init_path[i].x, init_path[i].y, init_path[i].theta, 0.0);

        if (i < T_) {
            control_traj_(0, i) = 0.0;
            control_traj_(1, i) = 0.0;
            slack_d_(i)         = rda_params_.min_sd() > 0 ? rda_params_.min_sd() : 0.1;
            dt_traj_(i)         = dt_;
        }
    }

    getObstaclesFromMap();

    // Initialize dual variables
    duals_.resize(obstacles_.size());
    for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
        auto& dual = duals_[obs_idx];
        dual.lam.setZero(obstacles_[obs_idx].edge_num, T_ + 1);
        dual.mu.setZero(4, T_ + 1);
        dual.z.setZero(T_);
        dual.xi.setZero(T_ + 1, 2);
        dual.zeta.setZero(T_);
    }

    if (start_pose_ptr == nullptr || goal_pose_ptr == nullptr) {
        LOG(ERROR) << "Start or goal pose is null";
        return false;
    }

    const int    max_iter       = rda_params_.max_iter() > 0 ? rda_params_.max_iter() : 5;
    const double iter_threshold = rda_params_.iter_threshold() > 0 ? rda_params_.iter_threshold() : 0.2;

    // Previous-iteration dual variables (for the dual residual)
    std::vector<RDADualVariables> duals_prev = duals_;

    for (int iter = 0; iter < max_iter; ++iter) {
        admm_iter_ = iter;
        if (!solveSU(*start_pose_ptr, *goal_pose_ptr)) {
            LOG(ERROR) << "Failed to solve SU subproblem at iteration " << iter;
            return false;
        }
        if (!solveLamMuZ()) {
            LOG(ERROR) << "Failed to solve LamMuZ subproblem at iteration " << iter;
            return false;
        }
        updateMultipliers();

        // ---- Primal residual: ||Hm|| across all obstacles and timesteps ----
        double resi_pri = 0.0;
        for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
            for (std::size_t t = 0; t < T_; ++t) {
                const Eigen::VectorXd& lam_t = duals_[obs_idx].lam.col(t + 1);
                const Eigen::VectorXd& mu_t  = duals_[obs_idx].mu.col(t + 1);
                double                 theta = state_traj_(2, t + 1);
                Eigen::Matrix2d        Rot;
                Rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
                const Eigen::MatrixXd& A_obs = obstacles_[obs_idx].A_list[t + 1];
                Eigen::Vector2d        Hm    = G_vehicle_.transpose() * mu_t + (A_obs * Rot).transpose() * lam_t;
                resi_pri += Hm.squaredNorm();
            }
        }
        resi_pri = std::sqrt(resi_pri);

        // ---- Dual residual: change in (lam, mu, z) w.r.t. previous iteration ----
        double resi_dual = 0.0;
        for (std::size_t obs_idx = 0; obs_idx < obstacles_.size(); ++obs_idx) {
            resi_dual += (duals_[obs_idx].lam - duals_prev[obs_idx].lam).squaredNorm();
            resi_dual += (duals_[obs_idx].mu - duals_prev[obs_idx].mu).squaredNorm();
            resi_dual += (duals_[obs_idx].z - duals_prev[obs_idx].z).squaredNorm();
        }
        resi_dual = std::sqrt(resi_dual);

        LOG(INFO) << "RDA iteration " << iter + 1 << " resi_pri=" << resi_pri << " resi_dual=" << resi_dual;

        // Save current duals for the next iteration's dual residual
        duals_prev = duals_;

        // Early stop only after at least 2 iterations (the first SU solve has no
        // obstacle penalty because all duals start at zero).
        if (iter >= 1 && resi_pri < iter_threshold && resi_dual < iter_threshold) {
            LOG(INFO) << "RDA early stop at iteration " << iter + 1;
            break;
        }
    }

    opt_traj_.clear();
    for (std::size_t i = 0; i <= T_; ++i) {
        opt_traj_.emplace_back(state_traj_(0, i), state_traj_(1, i), state_traj_(2, i), state_traj_(3, i));
    }

    return true;
}

}   // namespace backend
}   // namespace planning
