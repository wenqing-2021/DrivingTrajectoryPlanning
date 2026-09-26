// OCEAN: the ADMM planner of "OCEAN: An Openspace Collision-free Trajectory
// Planner for Autonomous Parking Based on ADMM" (arXiv:2403.05090v1).
//
// Algorithm 1 solves four blocks in order, each with the machinery it needs:
// the obstacle block (15) is one small second-order cone program per
// (obstacle, node) pair, the speed block (18) and the path block (20) are QPs,
// and the time block (19) is a closed-form scalar minimization.
#include "ocean_planner.h"
#include "eicos.hpp"
#include "logger.h"
#include "math/box2d.h"
#include "math/math_utils.h"
#include <algorithm>

namespace planning::backend {
namespace {
using Clock = std::chrono::steady_clock;

// Face normals of the rear-axle body box in the body frame: the rows of G in
// Eq. (2), ordered +x, +y, -x, -y.
Eigen::Matrix<double, 4, 2> BodyBoxNormals() {
    Eigen::Matrix<double, 4, 2> normals;
    normals << 1, 0, 0, 1, -1, 0, 0, -1;
    return normals;
}

double Milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Layout of the cone program of Eq. (15) for one (obstacle, node) pair:
//   rows [0, edges + 5)   lambda >= 0, mu >= 0, d <= -clearance
//   rows [edges + 5, + 6) rotated cone carrying the surrogate objective
//   rows [.., + 3)        the dual cone ||A^T lambda|| <= 1
constexpr int kDistanceRows  = 5;   // lambda(E) + mu(4) + d, plus the edges
constexpr int kObjectiveRows = 6;   // residual(3), d change, tau - 1, and 1 + tau
constexpr int kDualConeRows  = 3;   // s0 = 1 and the two components of A^T lambda
}   // namespace

// Sparse QP assembly and persistent solver workspaces.
void OCEANQuadratic::square(const Terms& terms, double offset, double weight) {
    for (const auto& [i, vi] : terms) {
        g[i] += 2 * weight * offset * vi;
        for (const auto& [j, vj] : terms) h.emplace_back(i, j, 2 * weight * vi * vj);
    }
}

void OCEANQuadratic::constraint(const Terms& terms, double lo, double hi) {
    const int row = static_cast<int>(lower.size());
    for (const auto& [i, v] : terms) a.emplace_back(row, i, v);
    lower.push_back(lo);
    upper.push_back(hi);
}

void OCEANQp::reset() {
    solver_.clearSolver();
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    ready_  = false;
    status_ = 0;
    solution_.resize(0);
}

bool OCEANQp::solve(const OCEANQuadratic& q) {
    hessian_.resize(q.n, q.n);
    hessian_.setFromTriplets(q.h.begin(), q.h.end());
    constraints_.resize(q.lower.size(), q.n);
    constraints_.setFromTriplets(q.a.begin(), q.a.end());
    gradient_ = q.g;
    lower_    = Eigen::Map<const Eigen::VectorXd>(q.lower.data(), q.lower.size());
    upper_    = Eigen::Map<const Eigen::VectorXd>(q.upper.data(), q.upper.size());
    if (!ready_) {
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setMaxIteration(20000);
        solver_.settings()->setAbsoluteTolerance(1e-7);
        solver_.settings()->setRelativeTolerance(1e-7);
        solver_.settings()->setPolish(true);
        solver_.data()->setNumberOfVariables(q.n);
        solver_.data()->setNumberOfConstraints(q.lower.size());
        if (!solver_.data()->setHessianMatrix(hessian_) || !solver_.data()->setGradient(gradient_) ||
            !solver_.data()->setLinearConstraintsMatrix(constraints_) || !solver_.data()->setLowerBound(lower_) ||
            !solver_.data()->setUpperBound(upper_) || !solver_.initSolver())
            return false;
        ready_ = true;
    } else if (!solver_.updateHessianMatrix(hessian_) || !solver_.updateGradient(gradient_) ||
               !solver_.updateLinearConstraintsMatrix(constraints_) || !solver_.updateBounds(lower_, upper_))
        return false;
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    status_ = static_cast<int>(solver_.getStatus());
    if (solver_.getStatus() != OsqpEigen::Status::Solved) return false;
    solution_ = solver_.getSolution();
    return solution_.allFinite();
}

void OCEANWorkspace::reset() {
    speed.reset();
    path.reset();
    stats.Clear();
    stats.set_backend("ocean");
    stats.set_control_source("optimized");
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
params::OCEANParams OCEANSolver::Defaults(params::OCEANParams p) {
#define DEFAULT(name, value) \
    if (!p.has_##name()) p.set_##name(value)
    DEFAULT(max_iter, 1000);
    DEFAULT(rho, 1000.0);
    DEFAULT(primal_tolerance, 1e-3);
    DEFAULT(step_tolerance, 1e-2);
    DEFAULT(w_state, 1.0);
    DEFAULT(w_control, 0.01);
    DEFAULT(w_state_diff, 0.001);
    DEFAULT(w_control_diff, 0.001);
    DEFAULT(w_distance, 0.1);
    DEFAULT(dt_min, 0.05);
    DEFAULT(dt_max, 0.2);
    DEFAULT(endpoint_position_tolerance, 0.05);
    DEFAULT(endpoint_heading_tolerance, 0.08);
    DEFAULT(min_clearance, 0.1);
    DEFAULT(max_inner_iter, 30);
    DEFAULT(inner_tolerance, 1e-4);
    DEFAULT(max_sqp_iter, 100);
    DEFAULT(trust_region, 0.2);
    DEFAULT(dynamics_tolerance, 1e-3);
    DEFAULT(workers, 1);
    DEFAULT(worker_timeout_ms, 5000);
    DEFAULT(time_limit, 15.0);
#undef DEFAULT
    return p;
}

// Rear-axle forward-Euler bicycle model without slip angle, the model OCEAN,
// RDA and OBCA all plan with. States are [x, y, theta, v], controls [delta, a].
Eigen::Vector4d OCEANSolver::Step(const Eigen::Vector4d& state, double steer, double accel, double dt,
                                  double wheelbase) {
    return state + dt * Eigen::Vector4d(state[3] * std::cos(state[2]),
                                        state[3] * std::sin(state[2]),
                                        state[3] * std::tan(steer) / wheelbase,
                                        accel);
}

OCEANSolver::OCEANSolver(const std::shared_ptr<vehicle_model::KinematicModel>& model,
                         const std::shared_ptr<map::Map>& map, const params::OCEANParams& params, double dt,
                         std::shared_ptr<OCEANWorkspace> workspace)
    : model_(model)
    , map_(map)
    , p_(Defaults(params))
    , w_(workspace ? std::move(workspace) : std::make_shared<OCEANWorkspace>())
    , dt_(dt)
    , budget_(p_.time_limit()) {}

bool OCEANSolver::expired() const {
    return Clock::now() >= deadline_;
}

bool OCEANSolver::fail(const std::string& reason) {
    w_->stats.set_status(reason);
    output_.clear();
    controls_output_.resize(0, 2);
    // A failed request still reports the iterate that was reached, so the caller
    // can see how close it came without mistaking it for a planned trajectory.
    if (x_.cols() == N_ + 1 && h_.size() == N_ && steer_.size() == N_) {
        double time = 0;
        for (int k = 0; k <= N_; ++k) {
            auto* s = w_->stats.add_candidate_states();
            s->set_x(x_(0, k) + origin_[0]);
            s->set_y(x_(1, k) + origin_[1]);
            s->set_theta(x_(2, k));
            s->set_v(x_(3, k));
            w_->stats.add_candidate_timestamps(time);
            if (k < N_) {
                auto* c = w_->stats.add_candidate_controls();
                c->set_accelerate(acc_[k]);
                c->set_steer_angle(steer_[k]);
                time += h_[k];
            }
        }
    }
    LOG(WARNING) << "OCEAN: " << reason << " iteration=" << w_->stats.iterations()
                 << " primal_squared=" << w_->stats.primal_squared() << " dynamics=" << w_->stats.dynamics_inf();
    return false;
}

// ---------------------------------------------------------------------------
// Obstacle block, Eq. (15): one dual certificate per (obstacle, node) pair
// ---------------------------------------------------------------------------
OCEANObstacle::OCEANObstacle(const Eigen::MatrixXd& halfspaces, const Eigen::VectorXd& bounds, int node_count)
    : A(halfspaces)
    , b(bounds) {
    reset(node_count);
}

Eigen::Vector2d OCEANObstacle::normal(int node) const {
    return A.transpose() * lambda.col(node);
}

void OCEANObstacle::reset(int node_count) {
    nodes.assign(node_count, Node{});
    lambda.setZero(A.rows(), node_count);
    mu.setZero(4, node_count);
    xi.setZero(2, node_count);
    d.setZero(node_count);
    zeta.setZero(node_count);
}

Eigen::Vector3d OCEANObstacle::residual(const Eigen::Vector4d& state, const Eigen::Vector4d& body, int k) const {
    const Eigen::Vector2d normal = A.transpose() * lambda.col(k);
    Eigen::Vector3d       r;
    r[0] = d[k] - body.dot(mu.col(k)) + (A * state.head<2>() - b).dot(lambda.col(k));
    r.tail<2>() =
        BodyBoxNormals().transpose() * mu.col(k) + common::math::RotationMatrix2d(state[2]).transpose() * normal;
    return r;
}

// KKT error of the current (lambda, mu, d), in residual units: every gradient
// term scales with rho, so the error is reported divided by it. With
// project = true the point first moves onto the feasible set of Eq. (11f)-(11i),
// so an accepted certificate never violates its own cone.
double OCEANObstacle::certificateError(const Eigen::Vector4d& state, const Eigen::Vector4d& body, int node, double rho,
                                       double weight, double clearance, bool project) {
    const int       edges  = A.rows();
    auto&           work   = nodes[node];
    Eigen::Vector2d normal = A.transpose() * lambda.col(node);
    if (project) {
        for (int j = 0; j < edges; ++j) lambda(j, node) = std::max(0.0, lambda(j, node));
        for (int j = 0; j < 4; ++j) mu(j, node) = std::max(0.0, mu(j, node));
        normal              = A.transpose() * lambda.col(node);
        const double length = normal.norm();
        if (length > 1.0) {
            lambda.col(node) /= length;
            normal /= length;
        }
        d[node]        = std::min(d[node], -clearance);
        work.projected = true;
    }
    // Gradient of the block objective of Eq. (15).
    Eigen::Vector3d residual = this->residual(state, body, node);
    residual[0] += zeta[node];
    residual.tail<2>() += xi.col(node);
    const Eigen::VectorXd distance = A * state.head<2>() - b;
    Eigen::VectorXd       gl =
        rho * (distance * residual[0] + A * common::math::RotationMatrix2d(state[2]) * residual.tail<2>());
    const Eigen::Vector4d gm    = rho * (-body * residual[0] + BodyBoxNormals() * residual.tail<2>());
    const double          gd    = rho * residual[0] + weight * std::exp(d[node]);
    double                error = 0;
    // The dual cone of Eq. (11h) is one smooth inequality over lambda.
    const double length = normal.norm();
    if (length > 1e-12) {
        const Eigen::VectorXd gradient = A * normal / length;
        // Fit its multiplier on the components of lambda that are strictly
        // inside their box; only those carry a stationarity condition.
        double numerator = 0, denominator = 0;
        for (int j = 0; j < edges; ++j) {
            if (lambda(j, node) <= 1e-7) continue;
            numerator -= gl[j] * gradient[j];
            denominator += gradient[j] * gradient[j];
        }
        double multiplier = denominator > 1e-14 ? numerator / denominator : 0.0;
        if (length > 1.0 - 1e-6) {
            multiplier = std::max(0.0, multiplier);
            gl += multiplier * gradient;   // active: remove the normal direction
            error = std::max(error, std::max(0.0, -multiplier));
        } else {
            error = std::max(error, std::max(0.0, multiplier));   // inactive: needs multiplier <= 0
        }
    }
    // A solver returns a variable that is meant to sit on its bound as a few
    // ulps above zero, so the active-set test must be relative to the scale of
    // that block; an absolute threshold would read a large certificate as
    // interior and then demand stationarity from a component that is at its bound.
    const double lambda_scale = 1e-7 * (1 + lambda.col(node).cwiseAbs().maxCoeff());
    for (int j = 0; j < edges; ++j)
        error = std::max(error, lambda(j, node) > lambda_scale ? std::abs(gl[j]) : std::max(0.0, -gl[j]));
    const double mu_scale = 1e-7 * (1 + mu.col(node).cwiseAbs().maxCoeff());
    for (int j = 0; j < 4; ++j)
        error = std::max(error, mu(j, node) > mu_scale ? std::abs(gm[j]) : std::max(0.0, -gm[j]));
    error = std::max(error, d[node] < -clearance - 1e-7 * (1 + std::abs(d[node])) ? std::abs(gd) : std::max(0.0, gd));
    return error / std::max(1.0, rho);
}

bool OCEANObstacle::solveNode(const Eigen::MatrixXd& states, const Eigen::Vector4d& body, int node, double rho,
                              double weight, double clearance, int max_inner, double tolerance,
                              std::chrono::steady_clock::time_point deadline) {
    auto&     work     = nodes[node];
    const int edges    = A.rows();
    const int stride   = edges + kDistanceRows;
    const int tau      = stride;
    const int vars     = tau + 1;
    const int rows     = stride + kObjectiveRows + kDualConeRows;
    auto&     cones    = work.cones;
    auto&     equality = work.equality;
    auto&     matrix   = work.matrix;
    auto&     rhs      = work.rhs;
    auto&     h        = work.h;
    auto&     c        = work.c;
    auto&     terms    = work.terms;
    cones.resize(2);
    cones[0] = kObjectiveRows;
    cones[1] = kDualConeRows;
    equality.resize(0, vars);
    matrix.resize(rows, vars);
    rhs.resize(0);
    c.setZero(vars);
    c[tau] = 1;
    terms.reserve(5 * edges + 40);
    // Keep lambda and re-minimize mu and d exactly for the new pose. When the
    // transported certificate still satisfies the KKT conditions of Eq. (15)
    // there is no need to call the cone solver at all. No obstacle is skipped:
    // an uncertified pair always falls through to the cone solver below.
    if (work.ready && lambda.col(node).minCoeff() >= -1e-8) {
        const Eigen::Vector2d normal = A.transpose() * lambda.col(node);
        const Eigen::Vector2d target =
            -common::math::RotationMatrix2d(states(2, node)).transpose() * normal - xi.col(node);
        mu.col(node) << std::max(0.0, target[0]), std::max(0.0, target[1]), std::max(0.0, -target[0]),
            std::max(0.0, -target[1]);
        const Eigen::VectorXd distance = A * states.col(node).head<2>() - b;
        d[node]                        = common::math::OptimalDistance(
            distance.dot(lambda.col(node)) - body.dot(mu.col(node)) + zeta[node], rho, weight, -clearance);
        work.projected = false;
        work.error     = certificateError(states.col(node), body, node, rho, weight, clearance, false);
        if (work.error <= tolerance) {
            work.status = 0;
            work.reused = true;
            return true;
        }
        work.reused = false;
    }
    // Eq. (15) keeps exp(d) exactly but needs an exponential cone, which the cone
    // solver does not provide. On d <= 0 the exponential has curvature at most
    // one, so the tangent quadratic below is a majorant and repeating the solve
    // converges to the minimizer of the exact objective.
    const double scale = std::sqrt(2 * rho);
    for (int inner = 0; inner < max_inner; ++inner) {
        if (std::chrono::steady_clock::now() >= deadline) {
            work.status = -100;
            return false;
        }
        terms.clear();
        h.setZero(rows);
        int row = 0;
        for (int j = 0; j < edges + 4; ++j) terms.emplace_back(row++, j, -1);
        terms.emplace_back(row, edges + 4, 1);
        h[row++] = -clearance;
        // ||[2 y, tau - 1]|| <= tau + 1  <=>  ||y||^2 <= tau.
        terms.emplace_back(row, tau, -1);
        h[row++]                       = 1;
        const Eigen::VectorXd distance = A * states.col(node).head<2>() - b;
        const Eigen::MatrixXd normals  = A * common::math::RotationMatrix2d(states(2, node));
        for (int j = 0; j < edges; ++j) terms.emplace_back(row, j, -scale * distance[j]);
        for (int j = 0; j < 4; ++j) terms.emplace_back(row, edges + j, scale * body[j]);
        terms.emplace_back(row, edges + 4, -scale);
        h[row++] = scale * zeta[node];
        for (int axis = 0; axis < 2; ++axis) {
            for (int j = 0; j < edges; ++j) terms.emplace_back(row, j, -scale * normals(j, axis));
            for (int j = 0; j < 4; ++j) terms.emplace_back(row, edges + j, -scale * BodyBoxNormals()(j, axis));
            h[row++] = scale * xi(axis, node);
        }
        const double slope = std::sqrt(2 * weight * std::exp(std::min(0.0, d[node])));
        terms.emplace_back(row, edges + 4, -slope);
        h[row++]     = -slope * d[node];
        c[edges + 4] = weight * std::exp(d[node]);
        terms.emplace_back(row, tau, -1);
        h[row++] = -1;
        h[row++] = 1;
        for (int axis = 0; axis < 2; ++axis) {
            for (int j = 0; j < edges; ++j) terms.emplace_back(row, j, -A(j, axis));
            ++row;
        }
        matrix.setFromTriplets(terms.begin(), terms.end());
        EiCOS::Solver solver(matrix, equality, c, h, rhs, cones);
        const auto    code = solver.solve();
        ++work.iterations;
        work.status = static_cast<int>(code);
        if (code != EiCOS::exitcode::optimal && code != EiCOS::exitcode::close_to_optimal) return false;
        const Eigen::VectorXd value = solver.solution();
        if (!value.allFinite()) {
            work.status = -101;
            return false;
        }
        const double previous_d = d[node];
        lambda.col(node)        = value.head(edges);
        mu.col(node)            = value.segment(edges, 4);
        d[node]                 = value[edges + 4];
        work.error              = certificateError(states.col(node), body, node, rho, weight, clearance, false);
        // The cone solver reports convergence for its own (scaled) tolerance, so
        // the returned point is checked against the cone and the box here.
        const Eigen::VectorXd slack            = h - matrix * value;
        const double          linear_violation = -slack.head(stride).minCoeff();
        double                cone_violation   = 0;
        for (int index = 0, first = stride; index < cones.size(); ++index) {
            const int dimension = cones[index];
            cone_violation      = std::max(
                cone_violation,
                (slack.segment(first + 1, dimension - 1).norm() - slack[first]) / (1 + std::abs(slack[first])));
            first += dimension;
        }
        work.violation = std::max(linear_violation, cone_violation);
        if (work.violation > 1e-6) {
            if (work.violation > 1e-3) {
                if (inner + 1 >= max_inner) {
                    work.status = -103;
                    return false;
                }
                continue;
            }
            work.projected = true;
            certificateError(states.col(node), body, node, rho, weight, clearance, true);
        }
        const double change = std::abs(d[node] - previous_d) / (1 + std::abs(d[node]));
        if (change <= tolerance || inner + 1 >= max_inner) {
            work.status = 0;
            work.ready  = true;
            return true;
        }
    }
    work.status = -101;   // surrogate iterations exhausted without a certificate
    return false;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
bool OCEANSolver::initialize(const vehicle_model::sdv_path& path, const vehicle_model::VehiclePose& start,
                             const vehicle_model::VehiclePose& goal) {
    const auto& vehicle = model_->GetVehicleParam();
    if (path.size() < 3 || !map_ || !std::isfinite(dt_) || dt_ <= 0) return fail("invalid_input");
    if (p_.max_iter() < 1 || p_.max_sqp_iter() < 1 || p_.max_inner_iter() < 1 || p_.workers() < 0 ||
        p_.worker_timeout_ms() <= 0)
        return fail("invalid_parameters");
    const std::vector<double> positive = {p_.rho(),
                                          p_.dt_min(),
                                          p_.dt_max(),
                                          p_.inner_tolerance(),
                                          p_.primal_tolerance(),
                                          p_.step_tolerance(),
                                          p_.trust_region(),
                                          p_.dynamics_tolerance(),
                                          p_.time_limit(),
                                          vehicle.wheel_base(),
                                          vehicle.length(),
                                          vehicle.width(),
                                          vehicle.max_acc(),
                                          vehicle.max_velocity(),
                                          vehicle.max_steer_angle()};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0) return fail("invalid_parameters");
    const std::vector<double> nonnegative = {p_.w_state(),
                                             p_.w_control(),
                                             p_.w_state_diff(),
                                             p_.w_control_diff(),
                                             p_.w_distance(),
                                             p_.min_clearance(),
                                             vehicle.rear_overhang(),
                                             p_.endpoint_position_tolerance(),
                                             p_.endpoint_heading_tolerance()};
    for (double value : nonnegative)
        if (!std::isfinite(value) || value < 0) return fail("invalid_parameters");
    // The endpoint box is the acceptance criterion the exported trajectory is
    // checked against, so a looser configuration is rejected rather than used.
    if (p_.dt_max() < p_.dt_min() || p_.endpoint_position_tolerance() > 0.05 ||
        p_.endpoint_heading_tolerance() > 0.08 || vehicle.max_steer_angle() >= 1.55 ||
        vehicle.rear_overhang() > vehicle.length())
        return fail("invalid_parameters");

    // Shift the problem to the start pose so every distance stays local, and take
    // the horizon from the frontend path.
    origin_ << start.x, start.y;
    start_ << 0, 0, common::math::NormalizeAngle(start.theta);
    N_ = path.size() - 1;
    x_.setZero(4, N_ + 1);
    eta_.setZero(4, N_);
    steer_.setZero(N_);
    acc_.setZero(N_);
    h_.setConstant(N_, std::clamp(dt_, p_.dt_min(), p_.dt_max()));
    for (int k = 0; k <= N_; ++k) {
        const double previous = k ? x_(2, k - 1) : start_[2];
        x_.col(k) << path[k].x - origin_[0], path[k].y - origin_[1],
            previous + common::math::NormalizeAngle(path[k].theta - previous), 0;
    }
    goal_ << goal.x - origin_[0], goal.y - origin_[1], x_(2, N_) + common::math::NormalizeAngle(goal.theta - x_(2, N_));
    if (!x_.allFinite() || !start_.allFinite() || !goal_.allFinite()) return fail("invalid_input");
    // Initial speed follows the path; the frontend path may contain reverse
    // segments, so the sign comes from its direction. The endpoints start at rest.
    for (int k = 1; k < N_; ++k) {
        const Eigen::Vector2d delta           = x_.col(k + 1).head<2>() - x_.col(k).head<2>();
        const double          signed_distance = delta.dot(Eigen::Vector2d(std::cos(x_(2, k)), std::sin(x_(2, k))));
        x_(3, k) = std::clamp(signed_distance / h_[k], -vehicle.max_velocity(), vehicle.max_velocity());
        if (std::abs(x_(3, k)) > 1e-6)
            steer_[k] = std::clamp(std::atan(vehicle.wheel_base() * (x_(2, k + 1) - x_(2, k)) / (h_[k] * x_(3, k))),
                                   -vehicle.max_steer_angle(),
                                   vehicle.max_steer_angle());
    }
    for (int k = 0; k < N_; ++k)
        acc_[k] = std::clamp((x_(3, k + 1) - x_(3, k)) / h_[k], -vehicle.max_acc(), vehicle.max_acc());
    reference_ = x_;
    body_ << vehicle.length() - vehicle.rear_overhang(), vehicle.width() / 2, vehicle.rear_overhang(),
        vehicle.width() / 2;

    // Obstacles enter as convex halfspaces A y <= b. Concave polygons are split so
    // the union of the pieces is the original obstacle, and no obstacle is dropped.
    std::vector<common::math::Polygon2d> polygons;
    for (const auto& polygon : map_->GetObsList()) {
        const auto pieces = polygon.DecomposeConvex();
        if (pieces.empty()) return fail("invalid_obstacle");
        polygons.insert(polygons.end(), pieces.begin(), pieces.end());
    }
    w_->obstacles.resize(polygons.size());
    for (std::size_t m = 0; m < polygons.size(); ++m) {
        auto&      obstacle = w_->obstacles[m];
        const auto points   = static_cast<int>(polygons[m].num_points());
        obstacle.A.resize(points, 2);
        obstacle.b.resize(points);
        for (int j = 0; j < points; ++j) {
            const auto&           edge = polygons[m].line_segments()[j];
            Eigen::Vector2d       normal(-edge.unit_direction().y(), edge.unit_direction().x());
            const Eigen::Vector2d point(edge.start().x(), edge.start().y());
            const Eigen::Vector2d center(polygons[m].center().x(), polygons[m].center().y());
            if (normal.dot(center - point) > 0) normal = -normal;   // outward normal
            obstacle.A.row(j) = normal.transpose();
            obstacle.b[j]     = normal.dot(point - origin_);
        }
        obstacle.reset(N_ + 1);
        obstacle.d.setConstant(-p_.min_clearance());
    }
    initial_.clear();
    for (int k = 0; k <= N_; ++k) {
        Eigen::Vector4d state = x_.col(k);
        state.head<2>() += origin_;
        initial_.push_back(state);
    }
    w_->stats.set_segments(N_);
    w_->stats.set_obstacles(w_->obstacles.size());
    return true;
}

// ---------------------------------------------------------------------------
// ADMM blocks
// ---------------------------------------------------------------------------
// Eq. (15) for every (obstacle, node) pair. The pairs are independent, so they
// are distributed over the worker pool; nothing else in an iteration is parallel.
bool OCEANSolver::solveObstacles() {
    const int steps = N_ + 1;
    // The per-round watchdog exists to stop waiting on a stuck worker; a serial
    // round is bounded by the request deadline alone.
    const auto round_deadline =
        p_.workers() <= 1 ? deadline_
                          : std::min(deadline_, Clock::now() + std::chrono::milliseconds(p_.worker_timeout_ms()));
    for (auto& obstacle : w_->obstacles)
        for (auto& node : obstacle.nodes) {
            node.status     = 0;
            node.iterations = 0;
            node.error      = 0;
            node.reused     = false;
        }
    const bool ok = w_->threads.run(w_->obstacles.size() * steps, p_.workers(), p_.worker_timeout_ms(), [&](int index) {
        return w_->obstacles[index / steps].solveNode(x_,
                                                      body_,
                                                      index % steps,
                                                      p_.rho(),
                                                      p_.w_distance(),
                                                      p_.min_clearance(),
                                                      p_.max_inner_iter(),
                                                      p_.inner_tolerance(),
                                                      round_deadline);
    });
    int        reused = 0, projected = 0;
    for (auto& obstacle : w_->obstacles)
        for (auto& node : obstacle.nodes) {
            w_->stats.set_inner_iterations(w_->stats.inner_iterations() + node.iterations);
            w_->stats.set_feasibility_violation(std::max(w_->stats.feasibility_violation(), node.violation));
            if (node.status != 0) {
                w_->stats.set_subproblem_status(node.status);
                w_->stats.set_certificate_error(std::max(w_->stats.certificate_error(), node.error));
            }
            if (node.reused) ++reused;
            if (node.projected) ++projected;
        }
    w_->stats.set_reused_certificates(w_->stats.reused_certificates() + reused);
    w_->stats.set_projected_certificates(w_->stats.projected_certificates() + projected);
    return ok;
}

// Eq. (18): speed and acceleration, with the rest of the iterate fixed.
bool OCEANSolver::solveSpeed() {
    const auto&    vehicle = model_->GetVehicleParam();
    const int      states  = N_ + 1;
    OCEANQuadratic q(states + N_);
    for (int k = 0; k < states; ++k) {
        const double limit = (k == 0 || k == N_) ? 0.0 : vehicle.max_velocity();
        q.bound(k, -limit, limit);
        q.square({{k, 1}}, -reference_(3, k), p_.w_state());
    }
    for (int k = 0; k < N_; ++k) {
        q.bound(states + k, -vehicle.max_acc(), vehicle.max_acc());
        q.square({{states + k, 1}}, 0, p_.w_control());
        q.square({{k + 1, 1}, {k, -1}}, 0, p_.w_state_diff() / h_[k]);
        if (k > 0) q.square({{states + k, 1}, {states + k - 1, -1}}, 0, p_.w_control_diff() / h_[k]);
        // Dynamics of the first three states, linear in v and a for fixed pose.
        const Eigen::Vector3d rate(std::cos(x_(2, k)), std::sin(x_(2, k)), std::tan(steer_[k]) / vehicle.wheel_base());
        for (int j = 0; j < 3; ++j)
            q.square({{k, h_[k] * rate[j]}}, x_(j, k) - x_(j, k + 1) + eta_(j, k), p_.rho() / 2);
        q.square({{k, 1}, {k + 1, -1}, {states + k, h_[k]}}, eta_(3, k), p_.rho() / 2);
    }
    if (!w_->speed.solve(q)) {
        w_->stats.set_subproblem_status(w_->speed.status());
        return false;
    }
    x_.row(3) = w_->speed.solution().head(states).transpose();
    acc_      = w_->speed.solution().tail(N_);
    return true;
}

// Eq. (19): the segment durations. With the other blocks fixed the objective is
// separable and each term is quadratic * h^2 + linear * h + reciprocal / h, so
// every segment is minimized exactly instead of being approximated by a QP.
bool OCEANSolver::solveTime() {
    const double wheelbase = model_->GetVehicleParam().wheel_base();
    for (int k = 0; k < N_; ++k) {
        const Eigen::Vector4d rate(x_(3, k) * std::cos(x_(2, k)),
                                   x_(3, k) * std::sin(x_(2, k)),
                                   x_(3, k) * std::tan(steer_[k]) / wheelbase,
                                   acc_[k]);
        const Eigen::Vector4d offset     = x_.col(k) - x_.col(k + 1) + eta_.col(k);
        double                reciprocal = p_.w_state_diff() * (x_.col(k + 1) - x_.col(k)).squaredNorm();
        if (k > 0)
            reciprocal +=
                p_.w_control_diff() * (std::pow(steer_[k] - steer_[k - 1], 2) + std::pow(acc_[k] - acc_[k - 1], 2));
        h_[k] = common::math::OptimalTimeStep(
            p_.rho() / 2 * rate.squaredNorm(), p_.rho() * rate.dot(offset), reciprocal, p_.dt_min(), p_.dt_max());
    }
    return h_.allFinite();
}

// Eq. (20) with the linearizations of Eq. (21): a sequential QP over the pose
// and the steering angle, with the three trigonometric auxiliary variables
// eliminated analytically.
bool OCEANSolver::solvePath() {
    const auto& vehicle            = model_->GetVehicleParam();
    const int   states             = N_ + 1;
    const int   steer_begin        = 3 * states;
    const int   variables          = steer_begin + N_;
    const auto  minimize_distances = [&] {
        for (auto& obstacle : w_->obstacles)
            for (int k = 0; k < states; ++k) {
                const double offset = (obstacle.A * x_.col(k).head<2>() - obstacle.b).dot(obstacle.lambda.col(k)) -
                                      body_.dot(obstacle.mu.col(k)) + obstacle.zeta[k];
                obstacle.d[k] = common::math::OptimalDistance(offset, p_.rho(), p_.w_distance(), -p_.min_clearance());
            }
    };
    minimize_distances();
    for (int inner = 0; inner < p_.max_sqp_iter(); ++inner) {
        if (expired()) return false;
        OCEANQuadratic q(variables);
        for (int k = 0; k < states; ++k)
            for (int j = 0; j < 3; ++j) {
                const int index = 3 * k + j;
                double    lo = x_(j, k) - p_.trust_region(), hi = x_(j, k) + p_.trust_region();
                if (k == 0 || k == N_) {
                    // Tighten the internal box slightly to absorb QP roundoff; the
                    // configured acceptance tolerance is never enlarged.
                    const double tolerance = std::max(
                        0.0, (j == 2 ? p_.endpoint_heading_tolerance() : p_.endpoint_position_tolerance()) - 1e-5);
                    const double target = (k == 0 ? start_[j] : goal_[j]);
                    lo                  = target - tolerance;
                    hi                  = target + tolerance;
                }
                q.bound(index, lo, hi);
                q.square({{index, 1}}, -reference_(j, k), p_.w_state());
                if (k < N_) q.square({{index, -1}, {index + 3, 1}}, 0, p_.w_state_diff() / h_[k]);
            }
        for (int k = 0; k < N_; ++k) {
            const int steering = steer_begin + k;
            q.bound(steering,
                    std::max(-vehicle.max_steer_angle(), steer_[k] - p_.trust_region()),
                    std::min(vehicle.max_steer_angle(), steer_[k] + p_.trust_region()));
            q.square({{steering, 1}}, 0, p_.w_control());
            if (k > 0) q.square({{steering, 1}, {steering - 1, -1}}, 0, p_.w_control_diff() / h_[k]);
            // Dynamics, linearized in the pose and the steering angle, with
            // sin(theta), cos(theta) and tan(delta) replaced by Eq. (21).
            const double theta = x_(2, k), distance = x_(3, k) * h_[k];
            const double cosine = std::cos(theta), sine = std::sin(theta);
            q.square({{3 * k, 1}, {3 * (k + 1), -1}, {3 * k + 2, -distance * sine}},
                     distance * (cosine + sine * theta) + eta_(0, k),
                     p_.rho() / 2);
            q.square({{3 * k + 1, 1}, {3 * (k + 1) + 1, -1}, {3 * k + 2, distance * cosine}},
                     distance * (sine - cosine * theta) + eta_(1, k),
                     p_.rho() / 2);
            const double slope = 1 + std::pow(std::tan(steer_[k]), 2), angular = distance / vehicle.wheel_base();
            q.square({{3 * k + 2, 1}, {3 * (k + 1) + 2, -1}, {steering, angular * slope}},
                     angular * (std::tan(steer_[k]) - slope * steer_[k]) + eta_(2, k),
                     p_.rho() / 2);
        }
        for (auto& obstacle : w_->obstacles)
            for (int k = 0; k < states; ++k) {
                const Eigen::Vector2d normal = obstacle.A.transpose() * obstacle.lambda.col(k);
                const double          offset = normal.dot(x_.col(k).head<2>()) - body_.dot(obstacle.mu.col(k)) -
                                      obstacle.b.dot(obstacle.lambda.col(k)) + obstacle.zeta[k];
                // Envelope derivatives of the exact minimization over d: this keeps
                // the effect of the distance variables without adding N * M columns.
                const double exponential = p_.w_distance() * std::exp(obstacle.d[k]);
                const double gradient    = p_.rho() * (obstacle.d[k] + offset);
                const double curvature   = obstacle.d[k] >= -p_.min_clearance() - 1e-12
                                               ? p_.rho()
                                               : p_.rho() * exponential / (p_.rho() + exponential);
                q.square({{3 * k, normal[0]}, {3 * k + 1, normal[1]}}, -normal.dot(x_.col(k).head<2>()), curvature / 2);
                q.g[3 * k] += gradient * normal[0];
                q.g[3 * k + 1] += gradient * normal[1];
                const Eigen::Vector2d rotated = common::math::RotationMatrix2d(x_(2, k)).transpose() * normal;
                const Eigen::Vector2d slope(rotated[1], -rotated[0]);
                const Eigen::Vector2d residual =
                    BodyBoxNormals().transpose() * obstacle.mu.col(k) + rotated - slope * x_(2, k) + obstacle.xi.col(k);
                for (int j = 0; j < 2; ++j) q.square({{3 * k + 2, slope[j]}}, residual[j], p_.rho() / 2);
            }
        if (!w_->path.solve(q)) {
            w_->stats.set_subproblem_status(w_->path.status());
            return false;
        }
        w_->stats.set_sqp_iterations(w_->stats.sqp_iterations() + 1);
        // Backtracking line search on the merit function, which is the objective
        // plus the ADMM penalties. Leaving the block entirely is a failure: the
        // iterate would otherwise be accepted without any decrease.
        const Eigen::MatrixXd        previous       = x_;
        const Eigen::VectorXd        previous_steer = steer_;
        std::vector<Eigen::VectorXd> previous_d;
        previous_d.reserve(w_->obstacles.size());
        for (const auto& obstacle : w_->obstacles) previous_d.push_back(obstacle.d);
        const double before = merit();
        double       alpha = 1, change = 0;
        double       after    = before;
        bool         accepted = false;
        for (int line = 0; line < 18; ++line, alpha *= 0.5) {
            change = 0;
            for (int k = 0; k < states; ++k)
                for (int j = 0; j < 3; ++j) {
                    const double step = alpha * (w_->path.solution()[3 * k + j] - previous(j, k));
                    x_(j, k)          = previous(j, k) + step;
                    change            = std::max(change, std::abs(step));
                }
            steer_ = previous_steer + alpha * (w_->path.solution().segment(steer_begin, N_) - previous_steer);
            change = std::max(change, (steer_ - previous_steer).lpNorm<Eigen::Infinity>());
            minimize_distances();
            after = merit();
            if (after <= before + 1e-9 * (1 + std::abs(before))) {
                accepted = true;
                break;
            }
        }
        if (!accepted) {
            w_->stats.set_subproblem_status(-202);
            x_     = previous;
            steer_ = previous_steer;
            for (std::size_t m = 0; m < previous_d.size(); ++m) w_->obstacles[m].d = previous_d[m];
            return false;
        }
        if (change <= p_.inner_tolerance() || std::abs(before - after) <= p_.inner_tolerance() * (1 + std::abs(before)))
            return true;
    }
    w_->stats.set_subproblem_status(-203);
    return false;
}

// ---------------------------------------------------------------------------
// Residuals, merit and validation
// ---------------------------------------------------------------------------
// Eq. (23a) without the indicator terms, plus the multiplier update of Eq. (22).
double OCEANSolver::updateResiduals() {
    const double wheelbase = model_->GetVehicleParam().wheel_base();
    double       sum = 0, dynamics_inf = 0;
    for (int k = 0; k < N_; ++k) {
        const Eigen::Vector4d r = Step(x_.col(k), steer_[k], acc_[k], h_[k], wheelbase) - x_.col(k + 1);
        sum += r.squaredNorm();
        dynamics_inf = std::max(dynamics_inf, r.lpNorm<Eigen::Infinity>());
        eta_.col(k) += r;
    }
    for (auto& obstacle : w_->obstacles)
        for (int k = 0; k <= N_; ++k) {
            const Eigen::Vector3d r = obstacle.residual(x_.col(k), body_, k);
            sum += r.squaredNorm();
            obstacle.zeta[k] += r[0];
            obstacle.xi.col(k) += r.tail<2>();
        }
    w_->stats.set_primal_squared(sum);
    w_->stats.set_dynamics_inf(dynamics_inf);
    return sum;
}

double OCEANSolver::objective() const {
    double value =
        p_.w_state() * (x_ - reference_).squaredNorm() + p_.w_control() * (steer_.squaredNorm() + acc_.squaredNorm());
    for (int k = 0; k < N_; ++k) {
        value += p_.w_state_diff() * (x_.col(k + 1) - x_.col(k)).squaredNorm() / h_[k];
        if (k > 0)
            value += p_.w_control_diff() *
                     (std::pow(steer_[k] - steer_[k - 1], 2) + std::pow(acc_[k] - acc_[k - 1], 2)) / h_[k];
    }
    for (const auto& obstacle : w_->obstacles) value += p_.w_distance() * obstacle.d.array().exp().sum();
    return value;
}

double OCEANSolver::merit() const {
    double       value     = objective();
    const double wheelbase = model_->GetVehicleParam().wheel_base();
    for (int k = 0; k < N_; ++k)
        value += p_.rho() / 2 *
                 (Step(x_.col(k), steer_[k], acc_[k], h_[k], wheelbase) - x_.col(k + 1) + eta_.col(k)).squaredNorm();
    for (const auto& obstacle : w_->obstacles)
        for (int k = 0; k <= N_; ++k) {
            Eigen::Vector3d r = obstacle.residual(x_.col(k), body_, k);
            r[0] += obstacle.zeta[k];
            r.tail<2>() += obstacle.xi.col(k);
            value += p_.rho() / 2 * r.squaredNorm();
        }
    return value;
}

// Independent check of the exported result: endpoint box, actuation and time
// limits, dense body-footprint collision and map bounds. Sampling is bounded by
// 0.05 m of body motion, so it is a check, not a swept-volume certificate.
bool OCEANSolver::validate() const {
    const auto& vehicle = model_->GetVehicleParam();
    if (!x_.allFinite() || !h_.allFinite() || !steer_.allFinite() || !acc_.allFinite()) return false;
    for (int j = 0; j < 3; ++j) {
        const double tolerance = j == 2 ? p_.endpoint_heading_tolerance() : p_.endpoint_position_tolerance();
        if (std::abs(x_(j, 0) - start_[j]) > tolerance + 1e-7 || std::abs(x_(j, N_) - goal_[j]) > tolerance + 1e-7)
            return false;
    }
    if (std::abs(x_(3, 0)) > 1e-3 || std::abs(x_(3, N_)) > 1e-3 ||
        x_.row(3).cwiseAbs().maxCoeff() > vehicle.max_velocity() + 1e-6 ||
        steer_.cwiseAbs().maxCoeff() > vehicle.max_steer_angle() + 1e-6 ||
        acc_.cwiseAbs().maxCoeff() > vehicle.max_acc() + 1e-6 || h_.minCoeff() < p_.dt_min() - 1e-7 ||
        h_.maxCoeff() > p_.dt_max() + 1e-7)
        return false;
    const double radius = std::hypot(std::max(body_[0], body_[2]), body_[1]);
    for (int k = 0; k <= N_; ++k) {
        int parts = 1;
        if (k < N_)
            parts = std::max(1,
                             static_cast<int>(std::ceil(((x_.col(k + 1) - x_.col(k)).head<2>().norm() +
                                                         radius * std::abs(x_(2, k + 1) - x_(2, k))) /
                                                        0.05)));
        for (int l = 0; l < parts; ++l) {
            Eigen::Vector4d pose = x_.col(k);
            if (k < N_) pose += double(l) / parts * (x_.col(k + 1) - x_.col(k));
            const double              offset = vehicle.length() / 2 - vehicle.rear_overhang();
            const common::math::Vec2d center(pose[0] + origin_[0] + offset * std::cos(pose[2]),
                                             pose[1] + origin_[1] + offset * std::sin(pose[2]));
            const common::math::Box2d box(center, pose[2], vehicle.length(), vehicle.width());
            for (const auto& obstacle : map_->GetObsList())
                if (obstacle.HasOverlap(box)) return false;
            std::vector<common::math::Vec2d> corners;
            box.GetAllCorners(&corners);
            for (const auto& corner : corners)
                if (!map_->IsInMap(corner.x(), corner.y())) return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Algorithm 1
// ---------------------------------------------------------------------------
bool OCEANSolver::Process(const vehicle_model::sdv_path& path, std::shared_ptr<vehicle_model::VehiclePose>& start,
                          std::shared_ptr<vehicle_model::VehiclePose>& goal) {
    // Every request starts from a clean workspace: multipliers, buffers, counters
    // and solver state from a previous plan must not leak into this one.
    w_->reset();
    output_.clear();
    initial_.clear();
    controls_output_.resize(0, 2);
    h_.resize(0);
    x_.resize(4, 0);
    steer_.resize(0);
    acc_.resize(0);
    N_ = 0;
    origin_.setZero();
    if (!std::isfinite(budget_) || budget_ <= 0) return fail("timeout");
    deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget_));
    if (!start || !goal || !model_) return fail("invalid_input");
    const auto prepare = Clock::now();
    if (!initialize(path, *start, *goal)) return false;
    w_->stats.set_prepare_ms(Milliseconds(prepare));

    for (int i = 0; i < p_.max_iter(); ++i) {
        if (expired()) return fail("timeout");
        w_->stats.set_iterations(i + 1);
        const Eigen::MatrixXd old_x      = x_;
        const Eigen::VectorXd old_h      = h_;
        const Eigen::VectorXd old_steer  = steer_;
        const Eigen::VectorXd old_acc    = acc_;
        auto                  start_time = Clock::now();
        if (!solveObstacles()) return fail(expired() ? "timeout" : "obstacle_failed");
        w_->stats.set_obstacle_ms(w_->stats.obstacle_ms() + Milliseconds(start_time));
        start_time = Clock::now();
        if (!solveSpeed()) return fail("speed_failed");
        w_->stats.set_speed_ms(w_->stats.speed_ms() + Milliseconds(start_time));
        start_time = Clock::now();
        if (!solveTime()) return fail("time_failed");
        w_->stats.set_time_ms(w_->stats.time_ms() + Milliseconds(start_time));
        start_time = Clock::now();
        if (!solvePath()) return fail(expired() ? "timeout" : "path_failed");
        w_->stats.set_path_ms(w_->stats.path_ms() + Milliseconds(start_time));
        start_time          = Clock::now();
        const double primal = updateResiduals();
        w_->stats.set_residual_ms(w_->stats.residual_ms() + Milliseconds(start_time));
        const double step = std::max({(x_ - old_x).cwiseAbs().maxCoeff(),
                                      (h_ - old_h).cwiseAbs().maxCoeff(),
                                      (steer_ - old_steer).cwiseAbs().maxCoeff(),
                                      (acc_ - old_acc).cwiseAbs().maxCoeff()});
        w_->stats.set_step_inf(step);
        LOG(INFO) << "OCEAN iteration=" << i + 1 << " primal_squared=" << primal
                  << " dynamics=" << w_->stats.dynamics_inf() << " step=" << step;
        // Eq. (23): both residuals are small. The dynamics term is checked on its
        // own because it is what the exported trajectory is validated against.
        if (primal <= p_.primal_tolerance() && step <= p_.step_tolerance() &&
            w_->stats.dynamics_inf() <= p_.dynamics_tolerance()) {
            start_time       = Clock::now();
            const bool valid = validate();
            w_->stats.set_validation_ms(Milliseconds(start_time));
            if (!valid) return fail("validation_failed");
            if (expired()) return fail("timeout");
            output_.clear();
            for (int k = 0; k <= N_; ++k) {
                Eigen::Vector4d state = x_.col(k);
                state.head<2>() += origin_;
                output_.push_back(state);
            }
            controls_output_.resize(N_, 2);
            controls_output_.col(0) = acc_;
            controls_output_.col(1) = steer_;
            w_->stats.set_status("success");
            return true;
        }
    }
    return fail("iteration_limit");
}

}   // namespace planning::backend
