#include "rda_planner.h"
#include "math/math_utils.h"
#include <cmath>
#include <limits>

#include <Eigen/Sparse>

namespace planning {
namespace backend {

std::size_t RDASolver::Idx(VariableIdx idx) {
    return static_cast<std::size_t>(idx);
}

double RDASolver::SoftThreshold(const double value, const double threshold) {
    if (value > threshold) { return value - threshold; }
    if (value < -threshold) { return value + threshold; }
    return 0.0;
}

bool RDASolver::solveStateControlStepWithEicos() {
    if (!qp_solver_ptr_) { qp_solver_ptr_ = std::make_unique<QPSolver>(); }

    const Eigen::Index var_num = static_cast<Eigen::Index>(admm_var_num_);
    const Eigen::Index eq_num  = static_cast<Eigen::Index>(admm_Aeq_.rows());

    // Compute dynamic H and g including Im and Hm ADMM penalty terms.
    Eigen::MatrixXd H_dense = admm_P_;
    Eigen::VectorXd g       = admm_q_;

    // Add original ADMM penalty from general inequalities G * x <= h
    const Eigen::SparseMatrix<double> G_sparse = admm_G_.sparseView();
    H_dense += admm_rho_ * Eigen::MatrixXd(G_sparse.transpose() * G_sparse);
    if (admm_G_.rows() > 0) { g += admm_G_.transpose() * (admm_lambda_ - admm_rho_ * admm_z_); }

    // Inject Im and Hm penalties.
    for (std::size_t obs_index = 0; obs_index < rda_obstacles_.size(); ++obs_index) {
        const auto& obs = rda_obstacles_[obs_index];
        for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
            const std::size_t state_index = time_index + 1;
            const std::size_t state_base  = state_index * (state_dim_ + control_dim_);

            // Recompute values
            // distance_var is technically getDistanceVariable(admm_x_, time_index) but we just use it implicitly in
            // jacobian
            const double             im = computeIm(obs_index, time_index, admm_x_);
            const Eigen::RowVector2d hm = computeHm(obs_index, time_index, admm_x_);

            // --- Inject Im Penalty ---
            // Im = obsA_lam^T * [x; y] - obsb_lam - mu^T * h - dis - z + zeta
            // d(Im)/d(x) = obsA_lam(0)
            // d(Im)/d(y) = obsA_lam(1)
            // d(Im)/d(dis) = -1
            Eigen::VectorXd J_im                   = Eigen::VectorXd::Zero(var_num);
            J_im(state_base + Idx(VariableIdx::X)) = obs.obsA_lam(state_index, 0);
            J_im(state_base + Idx(VariableIdx::Y)) = obs.obsA_lam(state_index, 1);
            J_im(getDistanceIndex(time_index))     = -1.0;

            // The penalty is 0.5 * ro1 * Im^2.
            // Using Gauss-Newton approximation: H += ro1 * J_im * J_im^T, g += ro1 * Im * J_im
            H_dense += im_penalty_weight_ * J_im * J_im.transpose();
            g += im_penalty_weight_ * im * J_im;

            // --- Inject Hm Penalty ---
            // Hm = mu^T * G + obsA_lam^T * R(theta) + xi
            // We differentiate Hm with respect to theta.
            const double theta     = admm_x_(state_base + Idx(VariableIdx::THETA));
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);

            // d(R)/d(theta) = [-sin, -cos; cos, -sin]
            Eigen::RowVector2d obsA_lam = obs.obsA_lam.row(state_index);
            Eigen::RowVector2d d_obsA_lam_rot(obsA_lam.x() * (-sin_theta) + obsA_lam.y() * cos_theta,
                                              obsA_lam.x() * (-cos_theta) + obsA_lam.y() * (-sin_theta));

            Eigen::MatrixXd J_hm                          = Eigen::MatrixXd::Zero(2, var_num);
            J_hm(0, state_base + Idx(VariableIdx::THETA)) = d_obsA_lam_rot.x();
            J_hm(1, state_base + Idx(VariableIdx::THETA)) = d_obsA_lam_rot.y();

            H_dense += hm_penalty_weight_ * J_hm.transpose() * J_hm;
            g += hm_penalty_weight_ * J_hm.transpose() * hm.transpose();
        }
    }

    Eigen::SparseMatrix<double> H = H_dense.sparseView();

    Eigen::SparseMatrix<double>         A(eq_num, var_num);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(admm_Aeq_.nonZeros()));
    for (Eigen::Index row = 0; row < admm_Aeq_.rows(); ++row) {
        for (Eigen::Index col = 0; col < admm_Aeq_.cols(); ++col) {
            const double val = admm_Aeq_(row, col);
            if (std::abs(val) > 1e-12) { triplets.emplace_back(row, col, val); }
        }
    }
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd lower_bound(eq_num);
    Eigen::VectorXd upper_bound(eq_num);
    lower_bound = admm_beq_;
    upper_bound = admm_beq_;

    qp_solver_ptr_->setVariableNums(static_cast<int>(var_num));
    qp_solver_ptr_->setConstraintNums(static_cast<int>(eq_num));

    if (!qp_solver_ptr_->Solve(H, g, A, lower_bound, upper_bound)) {
        LOG(WARNING) << "RDASolver::solveStateControlStepWithEicos failed: QP solver failed.";
        return false;
    }

    const Eigen::VectorXd* solution = qp_solver_ptr_->GetResult();
    if (solution == nullptr || solution->size() != var_num) {
        LOG(WARNING) << "RDASolver::solveStateControlStepWithEicos failed: unexpected QP solution size.";
        return false;
    }

    admm_x_ = *solution;
    return true;
}

bool RDASolver::ComputeObstacleHyperLane(const common::math::Polygon2d& obstacle, Eigen::MatrixXd& obstacle_A,
                                         Eigen::VectorXd& obstacle_b) {
    obstacle_A.resize(obstacle.num_points(), 2);
    obstacle_b.resize(obstacle.num_points());

    if (obstacle.num_points() < 3) { return false; }

    const auto line_segments = obstacle.line_segments();
    const auto center_pts    = obstacle.center();
    for (std::size_t j = 0; j < static_cast<std::size_t>(obstacle.num_points()); ++j) {
        const auto          edge_vector = line_segments[j].unit_direction();
        common::math::Vec2d normal_vector(-edge_vector.y(), edge_vector.x());
        common::math::Vec2d vec_to_center(center_pts.x() - line_segments[j].start().x(),
                                          center_pts.y() - line_segments[j].start().y());
        if (normal_vector.InnerProd(vec_to_center) > 0) { normal_vector *= -1; }
        obstacle_A(j, 0) = normal_vector.x();
        obstacle_A(j, 1) = normal_vector.y();
        obstacle_b(j)    = normal_vector.InnerProd(line_segments[j].start());
    }

    return true;
}

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
    , convergence_tolerance_(rda_params.convergence_tolerance() > 0.0 ? rda_params.convergence_tolerance() : 1e-4)
    , max_rda_iterations_(rda_params.max_iter() > 0 ? rda_params.max_iter() : 10)
    , penalty_weight_(rda_params.penalty_weight() > 0.0 ? rda_params.penalty_weight() : 200.0)
    , l1_weight_(rda_params.l1_weight() > 0.0 ? rda_params.l1_weight() : 1e-2)
    , use_warm_start_(rda_params.use_warm_start())
    , slack_gain_(8.0)
    , w_s_(rda_params.w_s() > 0.0 ? rda_params.w_s() : 1.0)
    , w_u_(rda_params.w_u() > 0.0 ? rda_params.w_u() : 1.0) {
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

    vehicle_G_ << 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0;
    vehicle_h_ << vehicle_length_ / 2.0, vehicle_width_ / 2.0, vehicle_length_ / 2.0, vehicle_width_ / 2.0;

    hm_penalty_weight_ = 1.0;
    im_penalty_weight_ = penalty_weight_;
    qp_solver_ptr_     = std::make_unique<QPSolver>();
}

// ─── Main interface ──────────────────────────────────────────────────────────

bool RDASolver::Process(const vehicle_model::sdv_path&               init_path,
                        std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                        std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    if (!setInitVariable(init_path, start_pose_ptr, goal_pose_ptr)) {
        LOG(WARNING) << "RDASolver::Process failed: initialization failed.";
        return false;
    }

    if (!iterativeSolve(init_path, start_pose_ptr, goal_pose_ptr)) {
        LOG(WARNING) << "RDASolver::Process failed: failed to formulate/solve optimization.";
        return false;
    }

    if (optimal_solution_.size() != static_cast<Eigen::Index>(total_var_dim_)) {
        LOG(WARNING) << "RDASolver::Process failed: invalid solution size.";
        return false;
    }

    states_result_.clear();
    controls_result_.clear();
    states_result_.reserve(N_);
    controls_result_.reserve(N_ > 0 ? (N_ - 1) : 0);
    for (std::size_t i = 0; i < N_; ++i) {
        const std::size_t base = i * (state_dim_ + control_dim_);
        Eigen::Vector4d   state;
        state << optimal_solution_[base + Idx(VariableIdx::X)], optimal_solution_[base + Idx(VariableIdx::Y)],
            common::math::NormalizeAngle(optimal_solution_[base + Idx(VariableIdx::THETA)]),
            optimal_solution_[base + Idx(VariableIdx::V)];
        states_result_.emplace_back(state);

        if (i + 1 < N_) {
            Eigen::Vector2d control;
            control << optimal_solution_[base + Idx(VariableIdx::ACCELERATION)],
                optimal_solution_[base + Idx(VariableIdx::STEER_ANGLE)];
            controls_result_.emplace_back(control);
        }
    }

    optimization_result_.states   = states_result_;
    optimization_result_.controls = controls_result_;

    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    if (setCostFunction(P, q)) {
        optimization_result_.cost = 0.5 * optimal_solution_.dot(P * optimal_solution_) + q.dot(optimal_solution_);
    } else {
        optimization_result_.cost = 0.0;
    }

    return true;
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

bool RDASolver::iterativeSolve(const vehicle_model::sdv_path&               init_path,
                               std::shared_ptr<vehicle_model::VehiclePose>& start_pose_ptr,
                               std::shared_ptr<vehicle_model::VehiclePose>& goal_pose_ptr) {
    (void)init_path;
    (void)start_pose_ptr;
    (void)goal_pose_ptr;

    if (N_ < 2 || total_var_dim_ == 0) {
        LOG(WARNING) << "RDASolver::iterativeSolve failed: invalid horizon or variable dimension.";
        return false;
    }

    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;
    Eigen::MatrixXd G;
    Eigen::VectorXd h;
    if (!constructCostAndConstraints(P, q, Aeq, beq, G, h)) {
        LOG(WARNING) << "RDASolver::iterativeSolve failed: constructCostAndConstraints failed.";
        return false;
    }

    if (!initializeAdmmWorkspace(P, q, Aeq, beq, G, h)) {
        LOG(WARNING) << "RDASolver::iterativeSolve failed: ADMM workspace initialization failed.";
        return false;
    }

    if (!runAdmmIterations()) {
        LOG(WARNING) << "RDASolver::iterativeSolve failed: ADMM iterations failed in x-step.";
        return false;
    }

    setOptimalResultFromAdmm();

    return true;
}

bool RDASolver::constructCostAndConstraints(Eigen::MatrixXd& P, Eigen::VectorXd& q, Eigen::MatrixXd& Aeq,
                                            Eigen::VectorXd& beq, Eigen::MatrixXd& G, Eigen::VectorXd& h) {
    if (!setCostFunction(P, q)) {
        LOG(WARNING) << "RDASolver::constructCostAndConstraints failed: setCostFunction failed.";
        return false;
    }

    Eigen::VectorXd x_lower;
    Eigen::VectorXd x_upper;
    if (!setVariableBounds(x_lower, x_upper)) {
        LOG(WARNING) << "RDASolver::constructCostAndConstraints failed: setVariableBounds failed.";
        return false;
    }

    Eigen::MatrixXd A_dyn;
    Eigen::VectorXd b_dyn;
    if (!setDynamicConstraints(A_dyn, b_dyn)) {
        LOG(WARNING) << "RDASolver::constructCostAndConstraints failed: setDynamicConstraints failed.";
        return false;
    }

    const std::size_t               n = total_var_dim_;
    std::vector<Eigen::RowVectorXd> eq_rows;
    std::vector<double>             eq_bounds;
    std::vector<Eigen::RowVectorXd> ineq_rows;
    std::vector<double>             ineq_bounds;

    eq_rows.reserve(static_cast<std::size_t>(A_dyn.rows()) + 8);
    eq_bounds.reserve(static_cast<std::size_t>(b_dyn.size()) + 8);
    ineq_rows.reserve(2 * n);
    ineq_bounds.reserve(2 * n);

    for (Eigen::Index row = 0; row < A_dyn.rows(); ++row) {
        eq_rows.emplace_back(A_dyn.row(row));
        eq_bounds.emplace_back(b_dyn(row));
    }

    for (std::size_t i = 0; i < n; ++i) {
        Eigen::RowVectorXd row            = Eigen::RowVectorXd::Zero(n);
        row(static_cast<Eigen::Index>(i)) = 1.0;

        const double lower = x_lower(static_cast<Eigen::Index>(i));
        const double upper = x_upper(static_cast<Eigen::Index>(i));
        if (std::abs(upper - lower) <= kNumericalEpsilon) {
            eq_rows.emplace_back(row);
            eq_bounds.emplace_back(upper);
            continue;
        }

        if (upper < kMaxConeWidth) {
            ineq_rows.emplace_back(row);
            ineq_bounds.emplace_back(upper);
        }
        if (lower > -kMaxConeWidth) {
            ineq_rows.emplace_back(-row);
            ineq_bounds.emplace_back(-lower);
        }
    }

    Aeq = Eigen::MatrixXd::Zero(eq_rows.size(), n);
    beq = Eigen::VectorXd::Zero(eq_rows.size());
    for (std::size_t row = 0; row < eq_rows.size(); ++row) {
        Aeq.row(row) = eq_rows[row];
        beq(row)     = eq_bounds[row];
    }

    G = Eigen::MatrixXd::Zero(ineq_rows.size(), n);
    h = Eigen::VectorXd::Zero(ineq_rows.size());
    for (std::size_t row = 0; row < ineq_rows.size(); ++row) {
        G.row(row) = ineq_rows[row];
        h(row)     = ineq_bounds[row];
    }

    return true;
}

bool RDASolver::initializeAdmmWorkspace(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& Aeq,
                                        const Eigen::VectorXd& beq, const Eigen::MatrixXd& G,
                                        const Eigen::VectorXd& h) {
    admm_P_   = P;
    admm_q_   = q;
    admm_Aeq_ = Aeq;
    admm_beq_ = beq;
    admm_G_   = G;
    admm_h_   = h;

    admm_var_num_        = total_var_dim_;
    admm_constraint_num_ = static_cast<std::size_t>(G.rows());
    admm_rho_            = std::max(penalty_weight_, 1e-3);
    admm_l1_weight_      = std::max(l1_weight_, 0.0);

    admm_x_      = use_warm_start_ && optimal_solution_.size() == static_cast<Eigen::Index>(admm_var_num_)
                       ? optimal_solution_
                       : initial_variables_;
    admm_z_      = (admm_G_ * admm_x_).cwiseMin(admm_h_);
    admm_lambda_ = Eigen::VectorXd::Zero(admm_constraint_num_);
    admm_mu_     = Eigen::VectorXd::Zero(admm_constraint_num_);
    admm_u_      = Eigen::VectorXd::Zero(admm_constraint_num_);

    cost_history_.clear();
    xi_.setZero(admm_constraint_num_);
    zeta_                           = admm_z_;
    optimization_result_.iterations = 0;

    return true;
}

bool RDASolver::runAdmmIterations() {
    for (int iter = 0; iter < max_rda_iterations_; ++iter) {
        optimization_result_.iterations = iter + 1;

        if (!solveStateControlStepWithEicos()) {
            LOG(WARNING) << "RDASolver::runAdmmIterations failed: solveStateControlStepWithEicos failed at "
                            "iteration "
                         << iter;
            return false;
        }

        double dual_norm = 0.0;
        if (!solveLambdaMuZStepWithEicos(dual_norm)) { return false; }

        const double primal_norm = updateXiZeta();

        double obj = 0.5 * admm_x_.dot(admm_P_ * admm_x_) + admm_q_.dot(admm_x_);
        for (std::size_t obs_index = 0; obs_index < rda_obstacles_.size(); ++obs_index) {
            for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
                const double             im = computeIm(obs_index, time_index, admm_x_);
                const Eigen::RowVector2d hm = computeHm(obs_index, time_index, admm_x_);
                obj += 0.5 * im_penalty_weight_ * im * im;
                obj += 0.5 * hm_penalty_weight_ * hm.squaredNorm();
            }
        }
        cost_history_.emplace_back(obj);

        if (primal_norm < convergence_tolerance_ && dual_norm < convergence_tolerance_) { break; }

        LOG(INFO) << "ADMM Iteration " << iter + 1 << ": cost = " << obj << ", primal_norm = " << primal_norm
                  << ", dual_norm = " << dual_norm;
    }

    return true;
}

bool RDASolver::solveLambdaMuZStepWithEicos(double& dual_residual) {
    dual_residual = 0.0;
    if (rda_obstacles_.empty()) { return true; }

    for (std::size_t obs_index = 0; obs_index < rda_obstacles_.size(); ++obs_index) {
        double obstacle_residual = 0.0;
        if (!solveObstacleLambdaMuZWithEicos(obs_index, obstacle_residual)) {
            LOG(WARNING) << "RDASolver::solveLambdaMuZStepWithEicos failed: solveObstacleLambdaMuZWithEicos failed for "
                            "obstacle index "
                         << obs_index;
            return false;
        }
        dual_residual += obstacle_residual;
    }

    dual_residual /= static_cast<double>(rda_obstacles_.size());
    updateObstacleDualProducts();
    return true;
}

bool RDASolver::solveObstacleLambdaMuZWithEicos(const std::size_t obs_index, double& residual) {
    residual = 0.0;
    if (obs_index >= rda_obstacles_.size()) { return false; }

    auto&              obs          = rda_obstacles_[obs_index];
    const Eigen::Index edge_num     = obs.A.rows();
    const Eigen::Index mu_dim       = static_cast<Eigen::Index>(kVehicleBoundaryNum);
    const Eigen::Index horizon      = static_cast<Eigen::Index>(N_);
    const Eigen::Index lam_base     = 0;
    const Eigen::Index mu_base      = edge_num * horizon;
    const Eigen::Index z_base       = mu_base + mu_dim * horizon;
    const Eigen::Index base_var_num = z_base + static_cast<Eigen::Index>(distance_dim_);

    struct ScalarQuadraticTerm {
        Eigen::RowVectorXd coeff;
        double             offset{0.0};
        double             weight{1.0};
    };

    std::vector<ScalarQuadraticTerm> quadratic_terms;
    quadratic_terms.reserve(distance_dim_ * 3);

    for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
        const std::size_t     state_index = time_index + 1;
        const std::size_t     state_base  = state_index * (state_dim_ + control_dim_);
        const Eigen::Vector2d trans(admm_x_(state_base + Idx(VariableIdx::X)),
                                    admm_x_(state_base + Idx(VariableIdx::Y)));
        const Eigen::VectorXd obs_A_trans = obs.A * trans;

        ScalarQuadraticTerm im_term;
        im_term.coeff = Eigen::RowVectorXd::Zero(base_var_num);
        for (Eigen::Index edge = 0; edge < edge_num; ++edge) {
            im_term.coeff(lam_base + state_index * edge_num + edge) = obs_A_trans(edge) - obs.b(edge);
        }
        for (Eigen::Index mu_index = 0; mu_index < mu_dim; ++mu_index) {
            im_term.coeff(mu_base + state_index * mu_dim + mu_index) = -vehicle_h_(mu_index);
        }
        im_term.coeff(z_base + static_cast<Eigen::Index>(time_index)) = -1.0;
        im_term.offset = -getDistanceVariable(admm_x_, time_index) + obs.zeta(time_index);
        im_term.weight = im_penalty_weight_;
        quadratic_terms.emplace_back(im_term);

        const double    theta = admm_x_(state_base + Idx(VariableIdx::THETA));
        Eigen::Matrix2d rot;
        rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
        const Eigen::MatrixXd obs_A_rot = obs.A * rot;

        for (Eigen::Index component = 0; component < 2; ++component) {
            ScalarQuadraticTerm hm_term;
            hm_term.coeff = Eigen::RowVectorXd::Zero(base_var_num);
            for (Eigen::Index edge = 0; edge < edge_num; ++edge) {
                hm_term.coeff(lam_base + state_index * edge_num + edge) = obs_A_rot(edge, component);
            }
            for (Eigen::Index mu_index = 0; mu_index < mu_dim; ++mu_index) {
                hm_term.coeff(mu_base + state_index * mu_dim + mu_index) = vehicle_G_(mu_index, component);
            }
            hm_term.offset = obs.xi(state_index, component);
            hm_term.weight = hm_penalty_weight_;
            quadratic_terms.emplace_back(hm_term);
        }
    }

    const Eigen::Index epigraph_var_num = static_cast<Eigen::Index>(quadratic_terms.size());
    const Eigen::Index ecos_var_num     = base_var_num + epigraph_var_num;

    std::vector<Eigen::Triplet<double>> triplets;
    std::vector<double>                 h_values;

    auto addLinearInequality = [&](const Eigen::Index var_index, const double coeff, const double bound) {
        const Eigen::Index row = static_cast<Eigen::Index>(h_values.size());
        triplets.emplace_back(row, var_index, coeff);
        h_values.emplace_back(bound);
    };

    for (Eigen::Index state = 0; state < horizon; ++state) {
        for (Eigen::Index edge = 0; edge < edge_num; ++edge) {
            addLinearInequality(lam_base + state * edge_num + edge, -1.0, 0.0);
        }
        for (Eigen::Index mu_index = 0; mu_index < mu_dim; ++mu_index) {
            addLinearInequality(mu_base + state * mu_dim + mu_index, -1.0, 0.0);
        }
    }
    for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
        addLinearInequality(z_base + static_cast<Eigen::Index>(time_index), -1.0, 0.0);
    }

    const Eigen::Index linear_ineq_num = static_cast<Eigen::Index>(h_values.size());
    const Eigen::Index norm_soc_num    = static_cast<Eigen::Index>(distance_dim_);
    const Eigen::Index soc_ineq_num    = 3 * epigraph_var_num + 3 * norm_soc_num;

    triplets.reserve(triplets.size() + 8 * quadratic_terms.size() + distance_dim_ * static_cast<std::size_t>(edge_num));

    for (Eigen::Index term = 0; term < epigraph_var_num; ++term) {
        const Eigen::Index t_index = base_var_num + term;
        const Eigen::Index row     = linear_ineq_num + 3 * term;
        triplets.emplace_back(row, t_index, -1.0);
        for (Eigen::Index col = 0; col < base_var_num; ++col) {
            const double val = quadratic_terms[term].coeff(col);
            if (std::abs(val) > 1e-12) { triplets.emplace_back(row + 1, col, -2.0 * val); }
        }
        triplets.emplace_back(row + 2, t_index, -1.0);
    }

    const Eigen::Index norm_soc_start = linear_ineq_num + 3 * epigraph_var_num;
    for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
        const std::size_t  state_index = time_index + 1;
        const Eigen::Index row         = norm_soc_start + 3 * static_cast<Eigen::Index>(time_index);
        for (Eigen::Index edge = 0; edge < edge_num; ++edge) {
            const Eigen::Index lam_index = lam_base + static_cast<Eigen::Index>(state_index) * edge_num + edge;
            triplets.emplace_back(row + 1, lam_index, -obs.A(edge, 0));
            triplets.emplace_back(row + 2, lam_index, -obs.A(edge, 1));
        }
    }

    Eigen::SparseMatrix<double> G_soc(linear_ineq_num + soc_ineq_num, ecos_var_num);
    G_soc.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd h_soc(linear_ineq_num + soc_ineq_num);
    for (Eigen::Index row = 0; row < linear_ineq_num; ++row) { h_soc(row) = h_values[row]; }
    for (Eigen::Index term = 0; term < epigraph_var_num; ++term) {
        const Eigen::Index row = linear_ineq_num + 3 * term;
        h_soc(row)             = 1.0;
        h_soc(row + 1)         = 2.0 * quadratic_terms[term].offset;
        h_soc(row + 2)         = -1.0;
    }
    for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
        const Eigen::Index row = norm_soc_start + 3 * static_cast<Eigen::Index>(time_index);
        h_soc(row)             = 1.0;
        h_soc(row + 1)         = 0.0;
        h_soc(row + 2)         = 0.0;
    }

    Eigen::VectorXd c_soc = Eigen::VectorXd::Zero(ecos_var_num);
    for (Eigen::Index term = 0; term < epigraph_var_num; ++term) {
        c_soc(base_var_num + term) = 0.5 * quadratic_terms[term].weight;
    }

    Eigen::SparseMatrix<double> Aeq_soc(0, ecos_var_num);
    Eigen::VectorXd             beq_soc(0);
    Eigen::VectorXi             qdims(epigraph_var_num + norm_soc_num);
    for (Eigen::Index i = 0; i < epigraph_var_num + norm_soc_num; ++i) { qdims(i) = 3; }

    EiCOS::Solver solver_soc(G_soc, Aeq_soc, c_soc, h_soc, beq_soc, qdims);
    const auto    exit_code = solver_soc.solve(false);
    if (!(exit_code == EiCOS::exitcode::optimal || exit_code == EiCOS::exitcode::close_to_optimal)) { return false; }

    const Eigen::VectorXd& solution = solver_soc.solution();
    if (solution.size() != ecos_var_num) { return false; }

    const Eigen::MatrixXd prev_lambda = obs.lambda;
    const Eigen::MatrixXd prev_mu     = obs.mu;
    const Eigen::VectorXd prev_z      = obs.z;

    for (Eigen::Index state = 0; state < horizon; ++state) {
        for (Eigen::Index edge = 0; edge < edge_num; ++edge) {
            obs.lambda(edge, state) = solution(lam_base + state * edge_num + edge);
        }
        for (Eigen::Index mu_index = 0; mu_index < mu_dim; ++mu_index) {
            obs.mu(mu_index, state) = solution(mu_base + state * mu_dim + mu_index);
        }
    }
    for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
        obs.z(time_index) = solution(z_base + static_cast<Eigen::Index>(time_index));
    }

    residual =
        (obs.lambda - prev_lambda).squaredNorm() + (obs.mu - prev_mu).squaredNorm() + (obs.z - prev_z).squaredNorm();
    return true;
}

double RDASolver::updateXiZeta() {
    std::vector<double> xi_values;
    std::vector<double> zeta_values;
    double              residual_norm_square = 0.0;
    xi_values.reserve(rda_obstacles_.size() * distance_dim_ * 2);
    zeta_values.reserve(rda_obstacles_.size() * distance_dim_);

    for (auto& obs : rda_obstacles_) {
        for (std::size_t time_index = 0; time_index < distance_dim_; ++time_index) {
            const std::size_t state_index = time_index + 1;
            const std::size_t state_base  = state_index * (state_dim_ + control_dim_);
            const double      theta       = admm_x_(state_base + Idx(VariableIdx::THETA));
            const double      cos_theta   = std::cos(theta);
            const double      sin_theta   = std::sin(theta);

            const Eigen::RowVector2d obsA_lam = obs.obsA_lam.row(state_index);
            const Eigen::RowVector2d obsA_lam_rot(obsA_lam.x() * cos_theta + obsA_lam.y() * sin_theta,
                                                  -obsA_lam.x() * sin_theta + obsA_lam.y() * cos_theta);
            const Eigen::RowVector2d mu_G    = obs.mu.col(state_index).transpose() * vehicle_G_;
            const Eigen::RowVector2d hm_base = mu_G + obsA_lam_rot;
            residual_norm_square += hm_base.squaredNorm();
            obs.xi.row(state_index) += hm_base;
            xi_values.emplace_back(obs.xi(state_index, 0));
            xi_values.emplace_back(obs.xi(state_index, 1));

            const Eigen::Vector2d trans(admm_x_(state_base + Idx(VariableIdx::X)),
                                        admm_x_(state_base + Idx(VariableIdx::Y)));
            const double          im_base =
                obsA_lam.dot(trans) - obs.obsb_lam(state_index) - obs.mu.col(state_index).dot(vehicle_h_);
            const double im_residual = im_base - getDistanceVariable(admm_x_, time_index) - obs.z(time_index);
            residual_norm_square += im_residual * im_residual;
            obs.zeta(time_index) += im_residual;
            zeta_values.emplace_back(obs.zeta(time_index));
        }
    }

    xi_ = Eigen::VectorXd::Zero(xi_values.size());
    for (std::size_t i = 0; i < xi_values.size(); ++i) { xi_(i) = xi_values[i]; }

    zeta_ = Eigen::VectorXd::Zero(zeta_values.size());
    for (std::size_t i = 0; i < zeta_values.size(); ++i) { zeta_(i) = zeta_values[i]; }

    return std::sqrt(residual_norm_square);
}

void RDASolver::setOptimalResultFromAdmm() {
    dual_variables_.resize(3 * admm_constraint_num_);
    dual_variables_.segment(0, admm_constraint_num_)                        = admm_lambda_;
    dual_variables_.segment(admm_constraint_num_, admm_constraint_num_)     = admm_mu_;
    dual_variables_.segment(2 * admm_constraint_num_, admm_constraint_num_) = admm_z_;

    optimal_solution_                = admm_x_;
    const double ineq_residual       = (admm_G_ * admm_x_ - admm_z_).norm();
    const double eq_residual         = admm_Aeq_.rows() == 0 ? 0.0 : (admm_Aeq_ * admm_x_ - admm_beq_).norm();
    optimization_result_.is_feasible = (ineq_residual + eq_residual < 10.0 * convergence_tolerance_);
}

bool RDASolver::initializeRDAWorkspace() {
    rda_obstacles_.clear();
    if (map_ptr_ == nullptr) { return false; }

    const auto& obs_list = map_ptr_->GetObsList();
    rda_obstacles_.reserve(obs_list.size());
    for (std::size_t obs_index = 0; obs_index < obs_list.size(); ++obs_index) {
        RDAObstacleWorkspace workspace;
        if (!ComputeObstacleHyperLane(obs_list[obs_index], workspace.A, workspace.b)) { continue; }

        const Eigen::Index edge_num = workspace.A.rows();
        workspace.lambda            = Eigen::MatrixXd::Constant(edge_num, N_, 0.1);
        workspace.mu                = Eigen::MatrixXd::Constant(kVehicleBoundaryNum, N_, 0.1);
        workspace.z                 = Eigen::VectorXd::Zero(distance_dim_);
        workspace.xi                = Eigen::MatrixXd::Zero(N_, 2);
        workspace.zeta              = Eigen::VectorXd::Zero(distance_dim_);
        workspace.obsA_lam          = Eigen::MatrixXd::Zero(N_, 2);
        workspace.obsb_lam          = Eigen::VectorXd::Zero(N_);
        rda_obstacles_.emplace_back(std::move(workspace));
    }

    updateObstacleDualProducts();
    return true;
}

void RDASolver::updateObstacleDualProducts() {
    for (auto& obs : rda_obstacles_) {
        for (std::size_t state_index = 0; state_index < N_; ++state_index) {
            obs.obsA_lam.row(state_index) = obs.lambda.col(state_index).transpose() * obs.A;
            obs.obsb_lam(state_index)     = obs.lambda.col(state_index).dot(obs.b);
        }
    }
}

double RDASolver::computeIm(const std::size_t obs_index, const std::size_t time_index,
                            const Eigen::VectorXd& variables) const {
    const auto&           obs         = rda_obstacles_[obs_index];
    const std::size_t     state_index = time_index + 1;
    const std::size_t     state_base  = state_index * (state_dim_ + control_dim_);
    const Eigen::Vector2d trans(variables(state_base + Idx(VariableIdx::X)),
                                variables(state_base + Idx(VariableIdx::Y)));

    return obs.obsA_lam.row(state_index).dot(trans) - obs.obsb_lam(state_index) -
           obs.mu.col(state_index).dot(vehicle_h_) - getDistanceVariable(variables, time_index) - obs.z(time_index) +
           obs.zeta(time_index);
}

Eigen::RowVector2d RDASolver::computeHm(const std::size_t obs_index, const std::size_t time_index,
                                        const Eigen::VectorXd& variables) const {
    const auto&       obs         = rda_obstacles_[obs_index];
    const std::size_t state_index = time_index + 1;
    const std::size_t state_base  = state_index * (state_dim_ + control_dim_);
    const double      theta       = variables(state_base + Idx(VariableIdx::THETA));
    const double      cos_theta   = std::cos(theta);
    const double      sin_theta   = std::sin(theta);

    const Eigen::RowVector2d obsA_lam = obs.obsA_lam.row(state_index);
    const Eigen::RowVector2d obsA_lam_rot(obsA_lam.x() * cos_theta + obsA_lam.y() * sin_theta,
                                          -obsA_lam.x() * sin_theta + obsA_lam.y() * cos_theta);

    return obs.mu.col(state_index).transpose() * vehicle_G_ + obsA_lam_rot + obs.xi.row(state_index);
}

double RDASolver::getDistanceVariable(const Eigen::VectorXd& variables, const std::size_t time_index) const {
    return variables(getDistanceIndex(time_index));
}

std::size_t RDASolver::getDistanceIndex(const std::size_t time_index) const {
    return distance_offset_ + time_index;
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

    N_               = init_path.size();
    distance_dim_    = N_ > 1 ? N_ - 1 : 0;
    distance_offset_ = N_ * (state_dim_ + control_dim_);
    total_var_dim_   = distance_offset_ + distance_dim_;
    initial_variables_.setZero(total_var_dim_);

    // Initialize state and control variables
    if (!initializeStateControl(init_path)) {
        LOG(WARNING) << "RDASolver::setInitVariable failed: initializeStateControl returned false.";
        return false;
    }

    for (std::size_t i = 0; i < distance_dim_; ++i) { initial_variables_[getDistanceIndex(i)] = kSafetyMargin; }

    // initialize dual variables
    if (!initializeDualVariables()) {
        LOG(WARNING) << "RDASolver::setInitVariable failed: initializeDualVariables returned false.";
        return false;
    }

    return true;
}

bool RDASolver::initializeStateControl(const vehicle_model::sdv_path& init_path) {
    auto clamp_value = [](double val, double lower, double upper) { return std::max(lower, std::min(val, upper)); };

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

    initial_states_.clear();
    initial_states_.reserve(N_);
    initial_controls_.clear();
    initial_controls_.reserve(N_);
    // Second pass: set state and control initialization.
    for (std::size_t i = 0; i < N_; ++i) {
        const std::size_t base                             = i * (state_dim_ + control_dim_);
        initial_variables_[base + Idx(VariableIdx::X)]     = init_path[i].x;
        initial_variables_[base + Idx(VariableIdx::Y)]     = init_path[i].y;
        initial_variables_[base + Idx(VariableIdx::THETA)] = init_path[i].theta;
        initial_variables_[base + Idx(VariableIdx::V)]     = v_estimates[i];

        initial_states_.emplace_back(init_path[i].x, init_path[i].y, init_path[i].theta, v_estimates[i]);

        double steer_init = 0.0;
        if (i + 1 < N_ && std::abs(v_estimates[i]) > 1e-3) {
            double d_theta = init_path[i + 1].theta - init_path[i].theta;
            d_theta        = common::math::NormalizeAngle(d_theta);
            steer_init     = std::atan(wheel_base_ * d_theta / (v_estimates[i] * dt_));
            steer_init     = clamp_value(steer_init, -max_steering_angle_, max_steering_angle_);
        }

        double acc_init = 0.0;
        if (i + 1 < N_) {
            acc_init = (v_estimates[i + 1] - v_estimates[i]) / dt_;
            acc_init = clamp_value(acc_init, -max_acceleration_, max_acceleration_);
        }

        initial_variables_[base + Idx(VariableIdx::ACCELERATION)] = acc_init;
        initial_variables_[base + Idx(VariableIdx::STEER_ANGLE)]  = steer_init;
        initial_controls_.emplace_back(acc_init, steer_init);
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

    if (!initializeRDAWorkspace()) {
        LOG(WARNING) << "RDASolver::initializeDualVariables failed: initializeRDAWorkspace returned false.";
        return false;
    }

    return true;
}

// ─── Private: constraints ────────────────────────────────────────────────────

bool RDASolver::setVariableBounds(Eigen::VectorXd& x_lower, Eigen::VectorXd& x_upper) {
    if (N_ == 0 || total_var_dim_ == 0) {
        LOG(WARNING) << "RDASolver::setVariableBounds failed: optimization variables are not initialized.";
        return false;
    }

    x_lower = Eigen::VectorXd::Constant(total_var_dim_, -kMaxConeWidth);
    x_upper = Eigen::VectorXd::Constant(total_var_dim_, kMaxConeWidth);

    for (std::size_t i = 0; i < N_; ++i) {
        const std::size_t base                         = i * (state_dim_ + control_dim_);
        x_lower[base + Idx(VariableIdx::V)]            = -max_velocity_;
        x_upper[base + Idx(VariableIdx::V)]            = max_velocity_;
        x_lower[base + Idx(VariableIdx::ACCELERATION)] = -max_acceleration_;
        x_upper[base + Idx(VariableIdx::ACCELERATION)] = max_acceleration_;
        x_lower[base + Idx(VariableIdx::STEER_ANGLE)]  = -max_steering_angle_;
        x_upper[base + Idx(VariableIdx::STEER_ANGLE)]  = max_steering_angle_;
    }

    // Clamp start/end states to initial guess for trajectory consistency.
    const std::size_t start_base = 0;
    const std::size_t end_base   = (N_ - 1) * (state_dim_ + control_dim_);

    for (const VariableIdx idx : {VariableIdx::X, VariableIdx::Y, VariableIdx::THETA, VariableIdx::V}) {
        const std::size_t id     = Idx(idx);
        x_lower[start_base + id] = initial_variables_[start_base + id];
        x_upper[start_base + id] = initial_variables_[start_base + id];
        x_lower[end_base + id]   = initial_variables_[end_base + id];
        x_upper[end_base + id]   = initial_variables_[end_base + id];
    }

    for (std::size_t i = 0; i < distance_dim_; ++i) {
        x_lower[getDistanceIndex(i)] = min_safety_distance_;
        x_upper[getDistanceIndex(i)] = max_safety_distance_;
    }

    return true;
}

bool RDASolver::setDynamicConstraints(Eigen::MatrixXd& A_constraint, Eigen::VectorXd& b_constraint) {
    if (N_ < 2 || initial_states_.size() != N_ || initial_controls_.size() != N_) {
        LOG(WARNING) << "RDASolver::setDynamicConstraints failed: invalid initialization state.";
        return false;
    }

    const std::size_t n_eq = (N_ - 1) * state_dim_;
    A_constraint           = Eigen::MatrixXd::Zero(n_eq, total_var_dim_);
    b_constraint           = Eigen::VectorXd::Zero(n_eq);

    for (std::size_t k = 0; k + 1 < N_; ++k) {
        const std::size_t row_base = k * state_dim_;
        const std::size_t base_k   = k * (state_dim_ + control_dim_);
        const std::size_t base_k1  = (k + 1) * (state_dim_ + control_dim_);

        const double th0    = initial_states_[k].z();
        const double v0     = initial_states_[k].w();
        const double delta0 = initial_controls_[k].y();

        const double cth          = std::cos(th0);
        const double sth          = std::sin(th0);
        const double tan_delta    = std::tan(delta0);
        const double cos_delta    = std::cos(delta0);
        const double cos_delta_sq = std::max(cos_delta * cos_delta, kNumericalEpsilon);

        // x_{k+1} = x_k + dt * v * cos(theta) linearized around (theta0, v0)
        {
            const std::size_t row                               = row_base;
            A_constraint(row, base_k1 + Idx(VariableIdx::X))    = 1.0;
            A_constraint(row, base_k + Idx(VariableIdx::X))     = -1.0;
            A_constraint(row, base_k + Idx(VariableIdx::V))     = -dt_ * cth;
            A_constraint(row, base_k + Idx(VariableIdx::THETA)) = dt_ * v0 * sth;
            b_constraint(row)                                   = dt_ * v0 * sth * th0;
        }

        // y_{k+1} = y_k + dt * v * sin(theta) linearized around (theta0, v0)
        {
            const std::size_t row                               = row_base + 1;
            A_constraint(row, base_k1 + Idx(VariableIdx::Y))    = 1.0;
            A_constraint(row, base_k + Idx(VariableIdx::Y))     = -1.0;
            A_constraint(row, base_k + Idx(VariableIdx::V))     = -dt_ * sth;
            A_constraint(row, base_k + Idx(VariableIdx::THETA)) = -dt_ * v0 * cth;
            b_constraint(row)                                   = -dt_ * v0 * cth * th0;
        }

        // theta_{k+1} = theta_k + dt * v/L * tan(delta), first-order linearization.
        {
            const std::size_t row                                     = row_base + 2;
            A_constraint(row, base_k1 + Idx(VariableIdx::THETA))      = 1.0;
            A_constraint(row, base_k + Idx(VariableIdx::THETA))       = -1.0;
            A_constraint(row, base_k + Idx(VariableIdx::V))           = -(dt_ / wheel_base_) * tan_delta;
            A_constraint(row, base_k + Idx(VariableIdx::STEER_ANGLE)) = -(dt_ / wheel_base_) * (v0 / cos_delta_sq);
            b_constraint(row) = (dt_ / wheel_base_) * (v0 / cos_delta_sq) * delta0;
        }

        // v_{k+1} = v_k + dt * a_k
        {
            const std::size_t row                                      = row_base + 3;
            A_constraint(row, base_k1 + Idx(VariableIdx::V))           = 1.0;
            A_constraint(row, base_k + Idx(VariableIdx::V))            = -1.0;
            A_constraint(row, base_k + Idx(VariableIdx::ACCELERATION)) = -dt_;
            b_constraint(row)                                          = 0.0;
        }
    }

    return true;
}

// ─── Private: RDA iteration ──────────────────────────────────────────────────

// ─── Private: obstacle / cost helpers ────────────────────────────────────────

bool RDASolver::setCostFunction(Eigen::MatrixXd& P, Eigen::VectorXd& q) {
    if (N_ == 0 || total_var_dim_ == 0) {
        LOG(WARNING) << "RDASolver::setCostFunction failed: optimization variables are not initialized.";
        return false;
    }

    P = Eigen::MatrixXd::Zero(total_var_dim_, total_var_dim_);
    q = Eigen::VectorXd::Zero(total_var_dim_);

    if (!setReferenceCost(P, q)) {
        LOG(WARNING) << "RDASolver::setCostFunction failed: setReferenceCost returned false.";
        return false;
    }

    for (std::size_t i = 0; i < distance_dim_; ++i) { q(getDistanceIndex(i)) -= slack_gain_; }

    return true;
}

bool RDASolver::setReferenceCost(Eigen::MatrixXd& P, Eigen::VectorXd& q) {
    if (initial_states_.size() != N_ || initial_controls_.size() != N_) {
        LOG(WARNING) << "RDASolver::setReferenceCost failed: initial_states or controls size does not match horizon.";
        return false;
    }

    // QP objective uses 0.5 * x^T P x + q^T x.
    // For w * ||x - x_ref||^2, each diagonal term is 2w and each linear term is -2w * x_ref.
    for (std::size_t i = 0; i < N_; ++i) {
        const std::size_t      base      = i * (state_dim_ + control_dim_);
        const Eigen::Vector4d& ref_state = initial_states_[i];

        const std::size_t x_index     = base + Idx(VariableIdx::X);
        const std::size_t y_index     = base + Idx(VariableIdx::Y);
        const std::size_t theta_index = base + Idx(VariableIdx::THETA);
        const std::size_t v_index     = base + Idx(VariableIdx::V);

        P(x_index, x_index) += 2.0 * w_s_;
        P(y_index, y_index) += 2.0 * w_s_;
        P(theta_index, theta_index) += 2.0 * w_s_;

        q(x_index) -= 2.0 * w_s_ * ref_state.x();
        q(y_index) -= 2.0 * w_s_ * ref_state.y();
        q(theta_index) -= 2.0 * w_s_ * ref_state.z();

        // Control variables and velocity (following Github's tracking design)
        if (i < N_ - 1) {
            const Eigen::Vector2d& ref_control = initial_controls_[i];
            const std::size_t      acc_index   = base + Idx(VariableIdx::ACCELERATION);
            const std::size_t      steer_index = base + Idx(VariableIdx::STEER_ANGLE);

            P(v_index, v_index) += 2.0 * w_u_;
            q(v_index) -= 2.0 * w_u_ * ref_state.w();

            P(acc_index, acc_index) += 2.0 * w_u_;
            q(acc_index) -= 2.0 * w_u_ * ref_control.x();

            P(steer_index, steer_index) += 2.0 * w_u_;
            q(steer_index) -= 2.0 * w_u_ * ref_control.y();
        }
    }

    return true;
}

}   // namespace backend
}   // namespace planning
